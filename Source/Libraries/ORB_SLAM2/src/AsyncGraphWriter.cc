/**
 * AsyncGraphWriter.cc
 *
 * See AsyncGraphWriter.h for the design rationale.
 */

#include "AsyncGraphWriter.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/sparse_optimizer.h"

namespace fs = std::filesystem;

namespace ORB_SLAM2
{

namespace
{

const char* phaseSuffix(AsyncGraphWriter::Phase p)
{
  return p == AsyncGraphWriter::Phase::Pre ? "pre" : "post";
}

const char* essentialKindStr(AsyncGraphWriter::EssentialKind k)
{
  return k == AsyncGraphWriter::EssentialKind::Sim3 ? "sim3" : "se3";
}

/// Serialise the optimizer to text via g2o::SparseOptimizer::save and
/// build a sidecar `.levels` body listing per-edge levels in the same
/// iteration order g2o uses.  Saved in lockstep so the line index in
/// .levels corresponds to the i-th edge record in .g2o.
void serialiseGraph(g2o::SparseOptimizer& optimizer,
                    std::string&          g2oOut,
                    std::string&          levelsOut)
{
  {
    std::stringstream ss;
    optimizer.save(ss);
    g2oOut = ss.str();
  }

  std::ostringstream lv;
  lv << "# edge_index level\n";
  std::size_t idx = 0;
  for (const auto& e : optimizer.edges())
  {
    auto* oe = static_cast<const g2o::OptimizableGraph::Edge*>(e);
    lv << idx << ' ' << oe->level() << '\n';
    ++idx;
  }
  levelsOut = lv.str();
}

} // anonymous namespace

// ─────────────────────────────────────────────────────────────────────────────

AsyncGraphWriter::AsyncGraphWriter(std::string prefix,
                                   int         localIntervalKF,
                                   int         essentialMinSpacingKF,
                                   bool        compress)
    : mEnabled(!prefix.empty())
    , mPrefix(std::move(prefix))
    , mLocalIntervalKF(localIntervalKF)
    , mEssentialMinSpacingKF(essentialMinSpacingKF)
    , mCompress(compress)
{
  if (!mEnabled)
    return;

  std::error_code ec;
  fs::create_directories(mPrefix, ec);
  if (ec)
  {
    std::cerr << "[AsyncGraphWriter] failed to create output directory '"
              << mPrefix << "': " << ec.message()
              << " — disabling dumps\n";
    mEnabled = false;
    return;
  }

  mThread = std::thread([this]() { writerMain(); });
}

AsyncGraphWriter::~AsyncGraphWriter()
{
  if (!mThread.joinable())
    return;

  {
    std::lock_guard<std::mutex> lk(mMutex);
    mStop.store(true);
  }
  mCv.notify_all();
  mThread.join();
}

// ─────────────────────────────────────────────────────────────────────────────

bool AsyncGraphWriter::checkLocalThrottle(int kfId)
{
  // A pre-dump bumps the throttle clock; the matching post-dump for
  // the same KF id is allowed unconditionally.
  if (kfId == mPendingLocalKf)
    return true;
  if (kfId - mLastLocalKf < mLocalIntervalKF)
    return false;
  mLastLocalKf      = kfId;
  mPendingLocalKf   = kfId;
  return true;
}

bool AsyncGraphWriter::checkEssentialThrottle(int kfId)
{
  if (kfId == mPendingEssentialKf)
    return true;
  if (kfId - mLastEssentialKf < mEssentialMinSpacingKF)
    return false;
  mLastEssentialKf      = kfId;
  mPendingEssentialKf   = kfId;
  return true;
}

bool AsyncGraphWriter::tryDumpLocal(g2o::SparseOptimizer& optimizer,
                                    int                   kfId,
                                    Phase                 phase)
{
  if (!mEnabled || !checkLocalThrottle(kfId))
    return false;

  // 6-digit zero-pad so names sort lexicographically as well as
  // numerically (kf000007 < kf000012 etc.).
  std::ostringstream name;
  name << "local_kf"
       << std::setw(6) << std::setfill('0') << kfId
       << '_' << phaseSuffix(phase) << ".g2o";
  snapshotAndEnqueue(optimizer, name.str());
  return true;
}

bool AsyncGraphWriter::tryDumpEssential(g2o::SparseOptimizer& optimizer,
                                        int                   kfId,
                                        EssentialKind         kind,
                                        Phase                 phase)
{
  if (!mEnabled || !checkEssentialThrottle(kfId))
    return false;

  std::ostringstream name;
  name << "essential_" << essentialKindStr(kind) << "_kf"
       << std::setw(6) << std::setfill('0') << kfId
       << '_' << phaseSuffix(phase) << ".g2o";
  snapshotAndEnqueue(optimizer, name.str());
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────

std::string AsyncGraphWriter::buildPath(const std::string& filename) const
{
  // Append .gz only at the path level (not in the in-memory `filename`
  // we receive from try*Dump) so the snapshot/serialise stage stays
  // unaware of compression.
  std::string suffixed = mCompress ? (filename + ".gz") : filename;
  return (fs::path(mPrefix) / suffixed).string();
}

void AsyncGraphWriter::snapshotAndEnqueue(g2o::SparseOptimizer& optimizer,
                                          const std::string&    g2oFilename)
{
  std::string g2oContent;
  std::string levelsContent;
  serialiseGraph(optimizer, g2oContent, levelsContent);

  PendingWrite g2oWrite{
      buildPath(g2oFilename),
      std::move(g2oContent)};
  PendingWrite levelsWrite{
      buildPath(g2oFilename + ".levels"),
      std::move(levelsContent)};

  {
    std::lock_guard<std::mutex> lk(mMutex);
    mQueue.push_back(std::move(g2oWrite));
    mQueue.push_back(std::move(levelsWrite));
  }
  mCv.notify_one();
}

void AsyncGraphWriter::writerMain()
{
  namespace io = boost::iostreams;

  for (;;)
  {
    PendingWrite job;
    {
      std::unique_lock<std::mutex> lk(mMutex);
      mCv.wait(lk, [this] { return mStop.load() || !mQueue.empty(); });
      if (mQueue.empty())
      {
        // Stop request and queue drained.
        if (mStop.load())
          return;
        continue;
      }
      job = std::move(mQueue.front());
      mQueue.pop_front();
    }

    // Open binary regardless of compression: gzip-compressed bytes are
    // binary, and even uncompressed g2o-text we want to write byte-for-
    // byte (no CRLF translation surprises on cross-platform repros).
    std::ofstream raw(job.path, std::ios::binary);
    if (!raw)
    {
      std::cerr << "[AsyncGraphWriter] failed to open '" << job.path
                << "' for writing\n";
      continue;
    }

    if (mCompress)
    {
      io::filtering_ostream out;
      out.push(io::gzip_compressor());
      out.push(raw);
      out << job.content;
      // filtering_ostream's destructor flushes the gzip filter and the
      // underlying ofstream, in that order.
    }
    else
    {
      raw << job.content;
    }
  }
}

} // namespace ORB_SLAM2
