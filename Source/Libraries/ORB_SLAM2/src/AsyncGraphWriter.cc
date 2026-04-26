/**
 * AsyncGraphWriter.cc
 *
 * See AsyncGraphWriter.h for the design rationale.
 */

#include "AsyncGraphWriter.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>

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

AsyncGraphWriter::AsyncGraphWriter(std::string prefix)
    : mEnabled(!prefix.empty())
    , mPrefix(std::move(prefix))
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
  if (kfId - mLastLocalKf < kLocalIntervalKF)
    return false;
  mLastLocalKf      = kfId;
  mPendingLocalKf   = kfId;
  return true;
}

bool AsyncGraphWriter::checkEssentialThrottle(int kfId)
{
  if (kfId == mPendingEssentialKf)
    return true;
  if (kfId - mLastEssentialKf < kEssentialMinSpacingKF)
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

  std::ostringstream name;
  name << "local_kf" << kfId << '_' << phaseSuffix(phase) << ".g2o";
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
  name << "essential_" << essentialKindStr(kind)
       << "_kf" << kfId << '_' << phaseSuffix(phase) << ".g2o";
  snapshotAndEnqueue(optimizer, name.str());
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────

std::string AsyncGraphWriter::buildPath(const std::string& filename) const
{
  return (fs::path(mPrefix) / filename).string();
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

    std::ofstream out(job.path);
    if (!out)
    {
      std::cerr << "[AsyncGraphWriter] failed to open '" << job.path
                << "' for writing\n";
      continue;
    }
    out << job.content;
  }
}

} // namespace ORB_SLAM2
