/**
 * AsyncGraphWriter.h
 *
 * Off-thread g2o graph dumper for ORB-SLAM2.  Produces serialised
 * snapshots of optimization graphs (LocalBundleAdjustment, OptimizeEssential
 * Graph) for offline analysis by the g2o_sampling library.
 *
 * Design
 * ------
 * Producer thread (Optimizer / LocalMapping / LoopClosing):
 *   tryDumpLocal / tryDumpEssential do throttling, then take a synchronous
 *   in-memory snapshot of the optimizer (calling g2o::SparseOptimizer::save
 *   into a std::stringstream — fast even for large graphs because it's just
 *   text concat; no disk I/O on this path).  The snapshot plus a sidecar
 *   `.levels` file (per-edge level info, lost by g2o's save) is enqueued.
 *
 * Consumer thread (single background worker):
 *   Pops snapshots from the queue and writes them to disk using the
 *   filename schema below.
 *
 * Throttling
 * ----------
 *   tryDumpLocal     — emits at most once every localIntervalKF KFs
 *                      (default 5; configurable).  Both pre and post
 *                      phases for a given KF id count as one trigger
 *                      event.
 *   tryDumpEssential — emits at most once every essentialMinSpacingKF
 *                      KFs (default 30; configurable).  Loop-closure
 *                      attempts can fire close together; this avoids
 *                      spam.
 *
 * Filename schema
 * ---------------
 *   <prefix>/local_kf<NNNNNN>_<phase>.<ext>
 *   <prefix>/essential_<kind>_kf<NNNNNN>_<phase>.<ext>
 *      <NNNNNN> — six-digit zero-padded KF id (so files sort
 *                 lexicographically and numerically)
 *      <kind>   — "sim3" (mono path) or "se3" (stereo / RGB-D path)
 *      <phase>  — "pre" (input graph before optimize) or "post"
 *      <ext>    — "g2o" or "g2o.levels" when compress=false
 *                 "g2o.gz" or "g2o.levels.gz" when compress=true
 *                 (default: true).  The .levels sidecar is gzipped
 *                 alongside the main g2o for consistency; absolute
 *                 saving is small but it keeps everything on the
 *                 same on/off switch.
 *
 * Disabled mode
 * -------------
 * Constructor with an empty `prefix` string disables every dump method
 * (all `tryDump*` calls return immediately).  No background thread is
 * started.  Production runs that don't pass a log prefix are unaffected.
 */

#ifndef ORB_SLAM2_ASYNC_GRAPH_WRITER_H
#define ORB_SLAM2_ASYNC_GRAPH_WRITER_H

#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

namespace g2o
{
class SparseOptimizer;
}

namespace ORB_SLAM2
{

class AsyncGraphWriter
{
 public:
  enum class EssentialKind { Sim3, SE3 };
  enum class Phase         { Pre, Post };

  /// Construct.  An empty @p prefix disables all dump methods (no
  /// background thread is started, every tryDump* call is a no-op).
  /// Otherwise, @p prefix is treated as a directory path relative to
  /// cwd; the directory is created if missing.
  ///
  /// @param localIntervalKF        Minimum KF id spacing between
  ///                               consecutive LocalBA dumps.  Default 5.
  /// @param essentialMinSpacingKF  Minimum KF id spacing between
  ///                               consecutive essential-graph dumps.
  ///                               Loop-closure attempts can fire close
  ///                               together so this is the de-spammer.
  ///                               Default 30.
  /// @param compress               If true (default), every output file
  ///                               passes through a gzip filter and
  ///                               gets a `.gz` suffix.  Roughly halves
  ///                               disk for ~free CPU on the writer
  ///                               thread.  Inspect with `zcat` or have
  ///                               the loader detect the suffix.
  explicit AsyncGraphWriter(std::string prefix,
                            int         localIntervalKF       = 5,
                            int         essentialMinSpacingKF = 30,
                            bool        compress              = true);

  /// Drains any queued writes and joins the background thread.
  ~AsyncGraphWriter();

  AsyncGraphWriter(const AsyncGraphWriter&)            = delete;
  AsyncGraphWriter& operator=(const AsyncGraphWriter&) = delete;

  /// Snapshot the optimizer (synchronous, in-memory) and queue a write
  /// to disk.  Throttled to at most once every kLocalIntervalKF
  /// keyframes.  Both pre and post phases for a single KF id count as
  /// one trigger.
  ///
  /// Returns true iff a snapshot was actually taken; useful for
  /// pairing pre+post calls (only call post if pre returned true).
  bool tryDumpLocal(g2o::SparseOptimizer& optimizer,
                    int                   kfId,
                    Phase                 phase);

  /// Same, for OptimizeEssentialGraph.  Throttled to at most once
  /// every kEssentialMinSpacingKF keyframes.
  bool tryDumpEssential(g2o::SparseOptimizer& optimizer,
                        int                   kfId,
                        EssentialKind         kind,
                        Phase                 phase);

  /// Whether dumping is enabled for this instance (i.e. constructor
  /// got a non-empty prefix).
  bool enabled() const { return mEnabled; }

 private:
  struct PendingWrite
  {
    std::string path;       ///< destination file path
    std::string content;    ///< serialised graph (g2o text or .levels body)
  };

  /// Compose the file path inside @p mPrefix for a given dump.
  std::string buildPath(const std::string& filename) const;

  /// Snapshot @p optimizer to a serialised g2o text representation
  /// AND a per-edge `.levels` body, queue both for async write.
  void snapshotAndEnqueue(g2o::SparseOptimizer& optimizer,
                          const std::string&    g2oFilename);

  /// Worker thread main.
  void writerMain();

  /// Throttle helpers — return true iff this id passes the spacing
  /// gate; updates the per-kind last-id state on success.
  bool checkLocalThrottle(int kfId);
  bool checkEssentialThrottle(int kfId);

  // ── config ──────────────────────────────────────────────────────────────
  bool                    mEnabled;
  std::string             mPrefix;
  int                     mLocalIntervalKF;
  int                     mEssentialMinSpacingKF;
  bool                    mCompress;

  // ── throttle state (only touched on producer thread) ────────────────────
  int                     mLastLocalKf      = -1000000;
  int                     mLastEssentialKf  = -1000000;
  // Pre/post pairing — the post call inherits the pre call's throttle
  // decision so we always get matching pairs.
  int                     mPendingLocalKf      = -1;
  int                     mPendingEssentialKf  = -1;

  // ── writer-thread plumbing ──────────────────────────────────────────────
  std::deque<PendingWrite>  mQueue;
  std::mutex                mMutex;
  std::condition_variable   mCv;
  std::atomic<bool>         mStop {false};
  std::thread               mThread;
};

} // namespace ORB_SLAM2

#endif // ORB_SLAM2_ASYNC_GRAPH_WRITER_H
