#include "syncai_mapping/pgos/sessions.h"

#include <fstream>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "nlohmann/json.hpp"

void writeSessions(
  const std::filesystem::path & dir, size_t total, const std::vector<size_t> & session_starts)
{
  nlohmann::ordered_json sessions = nlohmann::ordered_json::array();
  for (size_t k = 0; k < session_starts.size(); k++) {
    const size_t first = session_starts[k];
    const size_t end = k + 1 < session_starts.size() ? session_starts[k + 1] : total;
    if (end <= first) continue;  // a run that banked nothing
    nlohmann::ordered_json s;
    s["first"] = first;
    s["count"] = end - first;
    sessions.push_back(s);
  }

  const std::filesystem::path path = dir / kSessionsFile;
  std::error_code ec;
  if (sessions.size() <= 1) {
    std::filesystem::remove(path, ec);
    return;
  }
  nlohmann::ordered_json doc;
  doc["version"] = 1;
  doc["sessions"] = sessions;

  const std::filesystem::path tmp = path.string() + ".tmp";
  {
    std::ofstream out(tmp, std::ios::trunc);
    out << doc.dump(2) << "\n";
    out.close();
    if (!out) {
      std::filesystem::remove(tmp, ec);
      throw std::runtime_error("write failed: " + tmp.string());
    }
  }
  std::filesystem::rename(tmp, path, ec);
  if (ec) {
    const std::string why = ec.message();
    std::filesystem::remove(tmp, ec);
    throw std::runtime_error("cannot rename " + tmp.string() + ": " + why);
  }
}

bool readSessions(
  const std::filesystem::path & path, size_t total, std::vector<size_t> & starts, std::string & why)
{
  std::ifstream in(path);
  if (!in) {
    why = "cannot open it";
    return false;
  }
  const nlohmann::json doc = nlohmann::json::parse(in, nullptr, false);
  if (doc.is_discarded() || !doc.is_object()) {
    why = "not a JSON object";
    return false;
  }
  if (!doc.contains("version") || !doc["version"].is_number_integer() || doc["version"] != 1) {
    why = "not version 1";
    return false;
  }
  if (!doc.contains("sessions") || !doc["sessions"].is_array() || doc["sessions"].empty()) {
    why = "no sessions";
    return false;
  }
  size_t next = 0;
  std::vector<size_t> ret;
  for (const nlohmann::json & s : doc["sessions"]) {
    if (
      !s.is_object() || !s.contains("first") || !s.contains("count") ||
      !s["first"].is_number_unsigned() || !s["count"].is_number_unsigned()) {
      why = "a session without first / count";
      return false;
    }
    const size_t first = s["first"].get<size_t>();
    const size_t count = s["count"].get<size_t>();
    if (first != next || count == 0) {
      why = "the sessions do not follow one another";
      return false;
    }
    ret.push_back(first);
    next = first + count;
  }
  if (next != total) {
    why = "the sessions cover " + std::to_string(next) + " keyframes, poses.txt has " +
          std::to_string(total);
    return false;
  }
  starts = std::move(ret);
  return true;
}
