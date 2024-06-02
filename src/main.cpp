#include <iostream>

// #define CPPHTTPLIB_OPENSSL_SUPPORT
#include "httplib.h"
// #include "nlohmann/json.hpp"

std::string token{"jIy5181waJRDjqOHS7e9lVG3d9FayMj2LyWAk19pv0b"};

// // HTTP
// httplib::Client cli("http://cpp-httplib-server.yhirose.repl.co");

// HTTPS
httplib::Client cli("https://notify-api.line.me/api");

// -H 'Authorization: Bearer jIy5181waJRDjqOHS7e9lVG3d9FayMj2LyWAk19pv0b' -F
// 'message=test_ttttt'

int main(int argc, char const* argv[]) {
  std::cout << "start" << std::endl;

  std::string message{"message=test_ttttt"};
  httplib::Headers headers = {
      {"Method", "POST"},
      {"Content-Type", "application/x-www-form-urlencoded"},
      {"Authorization", "Bearer " + token}};

  // nlohmann::json jsonBody;
  // jsonBody["message"] = {
  //     {{"role", "user"}, {"content", "Hello! What's your name?"}}};
  // jsonBody["temperature"] = 0.7;

  // // JSONデータを文字列に変換
  // std::string body = jsonBody.dump();

  auto res = cli.Post("/notify", headers, message,
                      "application/x-www-form-urlencoded");
  // std::cout << "res->status" << res->status << std::endl;
  // std::cout << "res->body" << res->body << std::endl;

  return 0;
}
