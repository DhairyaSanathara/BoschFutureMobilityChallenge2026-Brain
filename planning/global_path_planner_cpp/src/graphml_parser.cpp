#include "global_path_planner_cpp/graphml_parser.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <sstream>
#include <unordered_map>

#include <tinyxml2.h>

namespace global_path_planner_cpp
{

namespace
{
using namespace tinyxml2;

// GraphML data-binding helper: every <data key="..."/> child carries one
// attribute by `key` (e.g. "d0", "d1"). The <key id="..." attr.name="..."/>
// declarations at the file head bind those short ids to human-readable
// attribute names. We resolve that mapping once and then read attributes by
// name regardless of the file's short-id choices.
struct KeyMap
{
  // short id (e.g. "d0") → semantic name ("x", "y", "speed_limit", …)
  std::unordered_map<std::string, std::string> for_node;
  std::unordered_map<std::string, std::string> for_edge;
};

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c){ return std::tolower(c); });
  return s;
}

// Best-effort string→double. Returns false on parse failure.
bool parse_double(const char * s, double & out)
{
  if (!s) return false;
  char * end = nullptr;
  const double v = std::strtod(s, &end);
  if (end == s) return false;
  out = v;
  return true;
}

bool parse_int(const char * s, int & out)
{
  if (!s) return false;
  char * end = nullptr;
  const long v = std::strtol(s, &end, 10);
  if (end == s) return false;
  out = static_cast<int>(v);
  return true;
}

// Build the key-id → attribute-name map by scanning the file's <key/> headers.
void build_key_map(const XMLElement * graphml_root, KeyMap & out)
{
  for (const XMLElement * k = graphml_root->FirstChildElement("key");
       k; k = k->NextSiblingElement("key"))
  {
    const char * id_   = k->Attribute("id");
    const char * name_ = k->Attribute("attr.name");
    const char * for_  = k->Attribute("for");
    if (!id_ || !name_ || !for_) continue;
    const std::string name = lower(name_);
    if (std::strcmp(for_, "node") == 0) out.for_node[id_] = name;
    else if (std::strcmp(for_, "edge") == 0) out.for_edge[id_] = name;
  }
}

// Pull all <data key="..."/> children of `el` and route them through `bind`,
// which takes (semantic_name, raw_value_cstring).
template <class Bind>
void each_data(const XMLElement * el,
               const std::unordered_map<std::string, std::string> & key_map,
               Bind && bind)
{
  for (const XMLElement * d = el->FirstChildElement("data");
       d; d = d->NextSiblingElement("data"))
  {
    const char * k = d->Attribute("key");
    if (!k) continue;
    auto it = key_map.find(k);
    if (it == key_map.end()) continue;
    bind(it->second, d->GetText());
  }
}

}  // namespace

bool load_graphml(const std::string & path, Graph & graph, std::string & error)
{
  XMLDocument doc;
  const XMLError rc = doc.LoadFile(path.c_str());
  if (rc != XML_SUCCESS) {
    std::ostringstream os;
    os << "tinyxml2 LoadFile('" << path << "') failed: "
       << XMLDocument::ErrorIDToName(rc);
    error = os.str();
    return false;
  }

  const XMLElement * root = doc.FirstChildElement("graphml");
  if (!root) { error = "no <graphml> root element"; return false; }

  KeyMap keys;
  build_key_map(root, keys);

  const XMLElement * gph = root->FirstChildElement("graph");
  if (!gph) { error = "no <graph> element under <graphml>"; return false; }

  // ----- Pass 1: ingest nodes ------------------------------------------------
  std::size_t node_count = 0;
  for (const XMLElement * n = gph->FirstChildElement("node");
       n; n = n->NextSiblingElement("node"))
  {
    GraphNode gn;
    const char * id_ = n->Attribute("id");
    if (!id_) continue;                       // skip silently — malformed node
    gn.id = id_;

    bool have_x = false, have_y = false;
    each_data(n, keys.for_node, [&](const std::string & name, const char * val) {
      if      (name == "x")    { have_x |= parse_double(val, gn.x); }
      else if (name == "y")    { have_y |= parse_double(val, gn.y); }
      else if (name == "type") { if (val) gn.type = val; }
    });
    if (!have_x || !have_y) continue;         // geometry-less node is unusable
    graph.add_node(std::move(gn));
    ++node_count;
  }

  // ----- Pass 2: ingest edges -----------------------------------------------
  std::size_t edge_count = 0;
  for (const XMLElement * e = gph->FirstChildElement("edge");
       e; e = e->NextSiblingElement("edge"))
  {
    GraphEdge ge;
    const char * src_ = e->Attribute("source");
    const char * tgt_ = e->Attribute("target");
    if (!src_ || !tgt_) continue;
    ge.source = src_;
    ge.target = tgt_;

    each_data(e, keys.for_edge, [&](const std::string & name, const char * val) {
      if      (name == "speed_limit")      { (void) parse_double(val, ge.speed_limit); }
      else if (name == "road_type" && val) { ge.road_type = val; }
      else if (name == "priority")         { (void) parse_int(val, ge.priority); }
      else if (name == "lane_direction" && val) { ge.lane_direction = val; }
      else if (name == "traffic_light_id" && val) { ge.traffic_light_id = val; }
    });

    graph.add_edge(std::move(ge));
    ++edge_count;
  }

  std::ostringstream os;
  os << "graphml ok: " << node_count << " nodes, " << edge_count << " edges";
  error = os.str();                            // not an error — caller treats as INFO
  return true;
}

}  // namespace global_path_planner_cpp
