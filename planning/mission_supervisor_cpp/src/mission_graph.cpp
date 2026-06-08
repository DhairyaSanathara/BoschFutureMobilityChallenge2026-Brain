#include "mission_supervisor_cpp/mission_graph.hpp"

#include <queue>
#include <sstream>
#include <unordered_map>

#include <tinyxml2.h>

namespace mission_supervisor_cpp
{

bool MissionGraph::load(const std::string & graphml_path, std::string & error)
{
  using namespace tinyxml2;
  XMLDocument doc;
  XMLError rc = doc.LoadFile(graphml_path.c_str());
  if (rc != XML_SUCCESS) {
    std::ostringstream os; os << "tinyxml2 load failed: " << static_cast<int>(rc)
                              << " path=" << graphml_path;
    error = os.str();
    return false;
  }
  XMLElement * root = doc.RootElement();
  if (!root) { error = "graphml has no root element"; return false; }
  XMLElement * graph = root->FirstChildElement("graph");
  if (!graph) { error = "graphml has no <graph> element"; return false; }

  // First pass: find the <key id> for x, y attributes (the BFMC file uses
  // "d0", "d1", "d2" with attr.name x/y/dotted respectively).
  std::string key_x, key_y;
  for (XMLElement * k = root->FirstChildElement("key"); k != nullptr;
       k = k->NextSiblingElement("key"))
  {
    const char * attr_name = k->Attribute("attr.name");
    const char * id        = k->Attribute("id");
    if (!attr_name || !id) continue;
    const std::string an(attr_name);
    if (an == "x") key_x = id;
    if (an == "y") key_y = id;
  }
  if (key_x.empty() || key_y.empty()) {
    error = "graphml missing x or y key declaration";
    return false;
  }

  // Pass 1 — nodes.
  for (XMLElement * n = graph->FirstChildElement("node"); n != nullptr;
       n = n->NextSiblingElement("node"))
  {
    const char * id = n->Attribute("id");
    if (!id) continue;
    GraphNode gn; gn.id = id;
    for (XMLElement * d = n->FirstChildElement("data"); d != nullptr;
         d = d->NextSiblingElement("data"))
    {
      const char * k = d->Attribute("key");
      const char * v = d->GetText();
      if (!k || !v) continue;
      const std::string kk = k;
      if (kk == key_x) gn.x = std::atof(v);
      if (kk == key_y) gn.y = std::atof(v);
    }
    nodes_.emplace(gn.id, std::move(gn));
  }

  // Pass 2 — edges. Directional; BFMC encodes one-way streets through pair direction.
  for (XMLElement * e = graph->FirstChildElement("edge"); e != nullptr;
       e = e->NextSiblingElement("edge"))
  {
    const char * src = e->Attribute("source");
    const char * tgt = e->Attribute("target");
    if (!src || !tgt) continue;
    adj_[src].insert(tgt);
    ++n_edges_;
  }
  return true;
}

bool MissionGraph::has_node(const std::string & id) const
{
  return nodes_.find(id) != nodes_.end();
}

bool MissionGraph::has_edge(const std::string & src, const std::string & tgt) const
{
  auto it = adj_.find(src);
  if (it == adj_.end()) return false;
  return it->second.count(tgt) > 0;
}

const GraphNode * MissionGraph::node(const std::string & id) const
{
  auto it = nodes_.find(id);
  return it == nodes_.end() ? nullptr : &it->second;
}

std::vector<std::string>
MissionGraph::shortest_path(const std::string & src, const std::string & tgt) const
{
  std::vector<std::string> out;
  if (!has_node(src) || !has_node(tgt)) return out;
  if (src == tgt) { out.push_back(src); return out; }

  std::unordered_map<std::string, std::string> parent;
  parent.reserve(nodes_.size());
  std::queue<std::string> q;
  q.push(src);
  parent[src] = "";

  while (!q.empty()) {
    const std::string cur = std::move(q.front());
    q.pop();
    auto it = adj_.find(cur);
    if (it == adj_.end()) continue;
    for (const std::string & nxt : it->second) {
      if (parent.count(nxt)) continue;
      parent[nxt] = cur;
      if (nxt == tgt) {
        // Reconstruct path back from tgt → src.
        std::vector<std::string> rev;
        std::string n = tgt;
        while (!n.empty()) {
          rev.push_back(n);
          auto pit = parent.find(n);
          if (pit == parent.end()) break;
          n = pit->second;
        }
        out.assign(rev.rbegin(), rev.rend());
        return out;
      }
      q.push(nxt);
    }
  }
  return out;   // empty if unreachable
}

}  // namespace mission_supervisor_cpp
