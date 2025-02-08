#include "json.hpp"
#include <fstream>
#include <string>
#include <vector>

using json = nlohmann::json;

struct UnitTestMemEntry
{
    uint16_t addr;
    uint8_t data;
};

struct UnitTestState
{
    uint16_t pc;
    uint8_t s;
    uint8_t a;
    uint8_t x;
    uint8_t y;
    uint8_t p;
    std::vector<UnitTestMemEntry> ram;
};

struct UnitTestCycle
{
    uint16_t addr;
    uint8_t data;
    int rw;
};

struct UnitTest
{
        std::string name;
        UnitTestState init;
        UnitTestState fin;
        std::vector<UnitTestCycle> cycles;
};

void from_json(const json& j, UnitTestMemEntry& r) {
    j[0].get_to(r.addr);
    j[1].get_to(r.data);
}

void from_json(const json& j, UnitTestState& s) {
    j.at("pc").get_to(s.pc);
    j.at("s").get_to(s.s);
    j.at("a").get_to(s.a);
    j.at("x").get_to(s.x);
    j.at("y").get_to(s.y);
    j.at("p").get_to(s.p);
    j.at("ram").get_to(s.ram);
}
void from_json(const json& j, UnitTestCycle& c) {
    j[0].get_to(c.addr);
    j[1].get_to(c.data);
    c.rw = j[2].get<std::string>().compare("read") == 0;
}
void from_json(const json& j, UnitTest& ut) {
    j.at("name").get_to(ut.name);
    j.at("initial").get_to(ut.init);
    j.at("final").get_to(ut.fin);
    j.at("cycles").get_to(ut.cycles);
}

std::vector<UnitTest> read_testset(const std::string jsonfile) {

    std::ifstream f(jsonfile);
    json jsontests = json::parse(f);

    std::vector<UnitTest> testset;
    for (auto& t : jsontests)
        testset.push_back(t);

    return testset;
}
