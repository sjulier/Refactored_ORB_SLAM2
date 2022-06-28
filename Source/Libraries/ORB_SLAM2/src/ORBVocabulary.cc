#include "ORBVocabulary.h"

using namespace ::std;

namespace ORB_SLAM2 {

ORBVocabulary::ORBVocabulary(int k, int L, DBoW2::WeightingType weighting,
                             DBoW2::ScoringType scoring)
    : ::OrbVocabulary(k, L, weighting, scoring) {}

bool ORBVocabulary::loadFromTextFile(const string &filename) {
  ifstream f;
  f.open(filename);

  if (f.eof())
    return false;

  m_words.clear();
  m_nodes.clear();

  string s;
  getline(f, s);
  stringstream ss;
  ss << s;
  ss >> m_k;
  ss >> m_L;
  int n1, n2;
  ss >> n1;
  ss >> n2;

  if (m_k < 0 || m_k > 20 || m_L < 1 || m_L > 10 || n1 < 0 || n1 > 5 ||
      n2 < 0 || n2 > 3) {
    cerr << "Vocabulary loading failure: This is not a correct text file!"
         << endl;
    return false;
  }

  m_scoring = (DBoW2::ScoringType)n1;
  m_weighting = (DBoW2::WeightingType)n2;
  createScoringObject();

  // Store where we are in the file; we'll return to this point later
  streampos pos = f.tellg();

  // Now we need to count the number of lines in the file to
  // preallocate the memory. Unfortunately this is messy, but fairly
  // fast
  string line;
  int nb_nodes;
  for (nb_nodes = 0; getline(f, line); nb_nodes++)
    ;

  // Add a bit of extra space just in case the last line isn't \n terminated
  nb_nodes += 2;

  cout << "Padded dictionary size = " << nb_nodes << endl;

  // Resize the memory pool and get a pointer to the start for convenience
  mvDictionaryMemoryPool.resize(F::L * nb_nodes);
  if (mvDictionaryMemoryPool.size() != F::L * nb_nodes) {
    cerr << "Could not resize the memory pool to " << F::L * nb_nodes
         << " bytes" << endl;
    exit(0);
  }
  unsigned char *memory_pool_pointer = mvDictionaryMemoryPool.data();

  // Clear the EOF bit and go back to the file, just after the header. The rest
  // of the file consists of the dicionary entries
  f.clear();
  f.seekg(pos);

  // nodes
  int expected_nodes =
      (int)((pow((double)m_k, (double)m_L + 1) - 1) / (m_k - 1));
  m_nodes.reserve(expected_nodes);

  m_words.reserve(pow((double)m_k, (double)m_L + 1));

  m_nodes.resize(1);
  m_nodes[0].id = 0;
  while (!f.eof()) {
    string snode;
    getline(f, snode);

    if (snode.size() == 0)
      continue;

    stringstream ssnode;
    ssnode << snode;

    int nid = m_nodes.size();
    m_nodes.resize(m_nodes.size() + 1);
    m_nodes[nid].id = nid;

    int pid;
    ssnode >> pid;
    m_nodes[nid].parent = pid;
    m_nodes[pid].children.push_back(nid);

    int nIsLeaf;
    ssnode >> nIsLeaf;

    stringstream ssd;
    for (int iD = 0; iD < F::L; iD++) {
      string sElement;
      ssnode >> sElement;
      ssd << sElement << " ";
    }
    m_nodes[nid].descriptor = cv::Mat(1, F::L, CV_8U, memory_pool_pointer);
    memory_pool_pointer += F::L;
    F::fromString(m_nodes[nid].descriptor, ssd.str());

    ssnode >> m_nodes[nid].weight;

    if (nIsLeaf > 0) {
      int wid = m_words.size();
      m_words.resize(wid + 1);

      m_nodes[nid].word_id = wid;
      m_words[wid] = &m_nodes[nid];
    } else {
      m_nodes[nid].children.reserve(m_k);
    }
  }

  return true;
}

// --------------------------------------------------------------------------

void ORBVocabulary::saveToTextFile(const string &filename) const {
  ofstream f;
  f.open(filename);
  f << m_k << " " << m_L << " "
    << " " << m_scoring << " " << m_weighting << endl;

  for (size_t i = 1; i < m_nodes.size(); i++) {
    const Node &node = m_nodes[i];

    f << node.parent << " ";
    if (node.isLeaf())
      f << 1 << " ";
    else
      f << 0 << " ";

    f << F::toString(node.descriptor) << " " << (double)node.weight << endl;
  }

  f.close();
}

bool ORBVocabulary::loadFromBinaryFile(const string &filename) {
  fstream f;
  f.open(filename.c_str(), ios_base::in | ios::binary);
  unsigned int nb_nodes, size_node;
  f.read((char *)&nb_nodes, sizeof(nb_nodes));
  f.read((char *)&size_node, sizeof(size_node));
  f.read((char *)&m_k, sizeof(m_k));
  f.read((char *)&m_L, sizeof(m_L));
  f.read((char *)&m_scoring, sizeof(m_scoring));
  f.read((char *)&m_weighting, sizeof(m_weighting));
  createScoringObject();

  m_words.clear();
  m_words.reserve(pow((double)m_k, (double)m_L + 1));
  m_nodes.clear();
  m_nodes.resize(nb_nodes + 1);
  m_nodes[0].id = 0;
#ifdef _WIN32
  char *buf = new char[size_node];
#else
  char buf[size_node];
#endif // _WIN32
  int nid = 1;

  // Resize the memory pool and get a pointer to the start for convenience
  mvDictionaryMemoryPool.resize(F::L * nb_nodes);

  if (mvDictionaryMemoryPool.size() != F::L * nb_nodes) {
    cerr << "Could not resize the memory pool to " << F::L * nb_nodes
         << " bytes" << endl;
    exit(0);
  }

  unsigned char *memory_pool_pointer = mvDictionaryMemoryPool.data();

  while (!f.eof()) {
    f.read(buf, size_node);
    m_nodes[nid].id = nid;
    // FIXME
    const int *ptr = (int *)buf;
    m_nodes[nid].parent = *ptr;
    // m_nodes[nid].parent = *(const int*)buf;
    m_nodes[m_nodes[nid].parent].children.push_back(nid);
    m_nodes[nid].descriptor = cv::Mat(1, F::L, CV_8U, memory_pool_pointer);
    memory_pool_pointer += F::L;
    memcpy(m_nodes[nid].descriptor.data, buf + 4, F::L);
    m_nodes[nid].weight = *(float *)(buf + 4 + F::L);
    if (buf[8 + F::L]) { // is leaf
      int wid = m_words.size();
      m_words.resize(wid + 1);
      m_nodes[nid].word_id = wid;
      m_words[wid] = &m_nodes[nid];
    } else
      m_nodes[nid].children.reserve(m_k);
    nid += 1;
  }
  f.close();

#ifdef _WIN32
  delete buf;
#endif // _WIN32
  return true;
}

// --------------------------------------------------------------------------

bool ORBVocabulary::saveToBinaryFile(const string &filename) const {
  fstream f;
  f.open(filename.c_str(), ios_base::out | ios::binary);
  unsigned int nb_nodes = m_nodes.size();
  float _weight;
  unsigned int size_node = sizeof(m_nodes[0].parent) + F::L * sizeof(char) +
                           sizeof(_weight) + sizeof(bool);
  f.write((char *)&nb_nodes, sizeof(nb_nodes));
  f.write((char *)&size_node, sizeof(size_node));
  f.write((char *)&m_k, sizeof(m_k));
  f.write((char *)&m_L, sizeof(m_L));
  f.write((char *)&m_scoring, sizeof(m_scoring));
  f.write((char *)&m_weighting, sizeof(m_weighting));
  for (size_t i = 1; i < nb_nodes; i++) {
    const Node &node = m_nodes[i];
    f.write((char *)&node.parent, sizeof(node.parent));
    f.write((char *)node.descriptor.data, F::L);
    _weight = node.weight;
    f.write((char *)&_weight, sizeof(_weight));
    bool is_leaf = node.isLeaf();
    f.write((char *)&is_leaf,
            sizeof(is_leaf)); // i put this one at the end for alignement....
  }
  f.close();
  return true;
}

// --------------------------------------------------------------------------

} // namespace ORB_SLAM2
