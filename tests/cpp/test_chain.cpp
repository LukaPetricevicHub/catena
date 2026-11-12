// M2: rank ordering, index and count gossip (with dropped frames), the link handshake.
#include <algorithm>
#include <vector>

#include "chain.h"
#include "doctest.h"
#include "fake_hal.h"
#include "robot.h"

using catena::ChainLink;
using catena::Rank;
using catena::outranks;

namespace {

const uint8_t kSingle = CATENA_FLAG_HEAD | CATENA_FLAG_TAIL;

// A line of link layers with a lossy radio between neighbours. ids[0] is the head.
struct Line {
  std::vector<ChainLink> nodes;
  uint32_t lcg = 2463534242u;
  uint64_t now_us = 0;
  int drop_percent = 0;

  explicit Line(const std::vector<uint8_t>& ids) {
    for (uint8_t id : ids) nodes.emplace_back(id);
    for (size_t i = 1; i < nodes.size(); ++i) join(i);
  }
  // Put node i's tongue on node i - 1's hub.
  void join(size_t i) {
    nodes[i].set_ahead(nodes[i - 1].id(), now_us);
    nodes[i - 1].set_behind(nodes[i].id(), 1, nodes[i].id(), now_us);
  }
  void cut(size_t i) {
    nodes[i].clear_ahead();
    nodes[i - 1].clear_behind();
  }
  bool dropped() {
    lcg = lcg * 1664525u + 1013904223u;
    return static_cast<int>((lcg >> 16) % 100u) < drop_percent;
  }
  // One step: every robot sends its two link frames, the radio loses some.
  void round() {
    now_us += 20000;
    struct Frame {
      size_t to;
      uint8_t src;
      uint8_t data[CATENA_FRAME_MAX];
      uint8_t len;
    };
    std::vector<Frame> air;
    for (size_t i = 0; i < nodes.size(); ++i) {
      Frame f = {};
      f.src = nodes[i].id();
      if (nodes[i].behind() != CATENA_NONE) {
        f.to = i + 1;
        f.len = nodes[i].make_ack(f.data, 1.0f, 0.0f);
        if (!dropped()) air.push_back(f);
      }
      if (nodes[i].ahead() != CATENA_NONE) {
        f.to = i - 1;
        f.len = nodes[i].make_up(f.data, false, 1.0f, false);
        if (!dropped()) air.push_back(f);
      }
    }
    for (const Frame& f : air) nodes[f.to].on_frame(f.src, f.data, f.len, now_us);
  }
  bool settled(size_t first, size_t last) const {
    uint8_t lowest = 255;
    for (size_t i = first; i <= last; ++i) lowest = std::min(lowest, nodes[i].id());
    for (size_t i = first; i <= last; ++i) {
      if (nodes[i].index() != i - first || nodes[i].size() != last - first + 1 ||
          nodes[i].chain_id() != lowest) {
        return false;
      }
    }
    return true;
  }
  int rounds_until_settled(size_t first, size_t last, int limit) {
    for (int n = 1; n <= limit; ++n) {
      round();
      if (settled(first, last)) return n;
    }
    return -1;
  }
};

const std::vector<uint8_t> kTwelve = {7, 3, 11, 0, 9, 4, 1, 10, 6, 2, 8, 5};

}  // namespace

TEST_CASE("the larger chain outranks the smaller, whatever their ids") {
  CHECK(outranks(Rank{3, 9}, Rank{2, 0}));
  CHECK_FALSE(outranks(Rank{2, 0}, Rank{3, 9}));
}

TEST_CASE("between chains of equal size the lower chain id outranks") {
  CHECK(outranks(Rank{4, 2}, Rank{4, 5}));
  CHECK_FALSE(outranks(Rank{4, 5}, Rank{4, 2}));
}

TEST_CASE("rank is a strict order, so two chains never chase each other") {
  std::vector<Rank> all;
  for (uint8_t size = 1; size <= 12; ++size) {
    for (uint8_t id = 0; id < 12; ++id) all.push_back(Rank{size, id});
  }
  for (const Rank& a : all) {
    CHECK_FALSE(outranks(a, a));
    for (const Rank& b : all) {
      const bool same = a.size == b.size && a.chain_id == b.chain_id;
      // Exactly one of two different chains outranks the other.
      CHECK((outranks(a, b) != outranks(b, a)) == !same);
      for (const Rank& c : {Rank{1, 11}, Rank{6, 3}, Rank{12, 0}}) {
        if (outranks(a, b) && outranks(b, c)) CHECK(outranks(a, c));
      }
    }
  }
}

TEST_CASE("a robot on its own is a chain of one: index 0, count 1, its own chain id") {
  ChainLink solo(6);
  CHECK(solo.index() == 0);
  CHECK(solo.size() == 1);
  CHECK(solo.chain_id() == 6);
}

TEST_CASE("index runs 0..11 and every robot learns count 12 and the lowest id") {
  Line line(kTwelve);
  const int rounds = line.rounds_until_settled(0, 11, 100);
  CHECK(rounds > 0);
  CHECK(rounds <= 12);  // one hop per step without losses
}

TEST_CASE("index and count still settle when the radio drops frames") {
  for (int percent : {5, 30, 60}) {
    Line line(kTwelve);
    line.drop_percent = percent;
    line.lcg += static_cast<uint32_t>(percent);
    const int rounds = line.rounds_until_settled(0, 11, 2000);
    CAPTURE(percent);
    CHECK(rounds > 0);
    // 5 s at 50 steps a second is the M2 limit for the simulated radio (5% loss).
    if (percent == 5) CHECK(rounds < 250);
    // Once settled, more lost frames do not disturb it.
    for (int i = 0; i < 200; ++i) {
      line.round();
      REQUIRE(line.settled(0, 11));
    }
  }
}

TEST_CASE("a chain that breaks recounts as two chains, and again when it rejoins") {
  Line line(kTwelve);
  line.drop_percent = 20;
  REQUIRE(line.rounds_until_settled(0, 11, 2000) > 0);
  line.cut(5);
  bool both = false;
  for (int i = 0; i < 2000 && !both; ++i) {
    line.round();
    both = line.settled(0, 4) && line.settled(5, 11);
  }
  CHECK(both);
  CHECK(line.nodes[5].index() == 0);
  CHECK(line.nodes[4].size() == 5);
  CHECK(line.nodes[11].size() == 7);
  line.join(5);
  CHECK(line.rounds_until_settled(0, 11, 2000) > 0);
}

TEST_CASE("link frames from a robot that is not the linked neighbour are ignored") {
  Line line({4, 2, 8});
  REQUIRE(line.rounds_until_settled(0, 2, 50) > 0);
  uint8_t forged[CATENA_FRAME_MAX];
  ChainLink stranger(1);
  stranger.set_behind(2, 1, 2, 0);
  const uint8_t len = stranger.make_ack(forged, 1.0f, 0.0f);
  CHECK_FALSE(line.nodes[1].on_frame(1, forged, len, line.now_us));
  CHECK(line.settled(0, 2));
}

TEST_CASE("a head seeks the tail of a higher-ranked chain") {
  FakeHal hal;
  catena::Robot robot(5, hal.p);
  const float ref = hal.p.ir_reference_range_mm;
  hal.inbox.push_back(status_flash(3, 0.4f, 180.0f, ref, 3, 1, 0, kSingle));
  robot.step(hal);
  hal.inbox.clear();
  robot.step(hal);
  CHECK(hal.debug.state == CATENA_STATE_SEEK);
  CHECK(hal.debug.target == 3);
  CHECK(hal.turn() > 0.0f);  // towards the tail on its left
}

TEST_CASE("a head never seeks a lower-ranked chain") {
  FakeHal hal;
  catena::Robot robot(3, hal.p);
  hal.inbox.push_back(status_flash(5, 2.5f, 180.0f, hal.p.ir_reference_range_mm, 5, 1, 0, kSingle));
  robot.step(hal);
  hal.inbox.clear();
  robot.step(hal);
  CHECK(hal.debug.state == CATENA_STATE_WANDER);
  CHECK(hal.debug.target == CATENA_NONE);
}

TEST_CASE("size beats id: a single robot seeks the tail of a pair with higher ids") {
  FakeHal hal;
  catena::Robot robot(1, hal.p);
  const float ref = hal.p.ir_reference_range_mm;
  // Robot 9 is the tail of chain {8, 9}; robot 8 is its head.
  hal.inbox.push_back(status_flash(9, -0.5f, 170.0f, ref, 8, 2, 1, CATENA_FLAG_TAIL, CATENA_NONE, 8));
  robot.step(hal);
  hal.inbox.clear();
  robot.step(hal);
  CHECK(hal.debug.state == CATENA_STATE_SEEK);
  CHECK(hal.debug.target == 9);
}

TEST_CASE("a seeker that sees only the middle of a chain works along it towards the tail") {
  FakeHal hal;
  catena::Robot robot(1, hal.p);
  const float ref = hal.p.ir_reference_range_mm;
  // Chain {8 head, 9, 10 tail}: 9 is close, 10 is further left; 9 names 10 as behind it.
  hal.inbox.push_back(status_flash(9, 0.0f, 150.0f, ref, 8, 3, 1, 0, CATENA_NONE, 8, 10));
  hal.inbox.push_back(status_flash(10, 0.5f, 190.0f, ref, 8, 3, 2, CATENA_FLAG_TAIL, CATENA_NONE, 9));
  robot.step(hal);
  hal.inbox.clear();
  robot.step(hal);
  CHECK(hal.debug.state == CATENA_STATE_SEEK);
  CHECK(hal.debug.target == 10);
}

TEST_CASE("a tail answers LINK_ACK only once its hall sensor fires") {
  FakeHal hal;
  catena::Robot tail(2, hal.p);
  const float ref = hal.p.ir_reference_range_mm;
  // Robot 6 stands one pitch behind robot 2 and asks to link.
  hal.inbox.push_back(status_flash(6, 3.1f, hal.p.pitch_mm, ref, 6, 1, 0, kSingle, 2));
  hal.inbox.push_back(unicast(6, {CATENA_FRAME_LINK_REQ, 6, 1}));
  tail.step(hal);
  CHECK(hal.sent_to(6, CATENA_FRAME_LINK_ACK) == 0);
  CHECK(hal.debug.behind == CATENA_NONE);

  hal.inbox.clear();
  hal.inbox.push_back(unicast(6, {CATENA_FRAME_LINK_REQ, 6, 1}));
  hal.hall_value = 1.0f;
  hal.sent.clear();
  tail.step(hal);
  CHECK(hal.sent_to(6, CATENA_FRAME_LINK_ACK) == 1);
  CHECK(hal.debug.behind == 6);
  CHECK(hal.debug.chain_size == 2);
  CHECK(hal.debug.index == 0);
  CHECK(hal.debug.chain_id == 2);
}

TEST_CASE("a link request from a robot never seen flashing is ignored") {
  FakeHal hal;
  catena::Robot tail(2, hal.p);
  hal.inbox.push_back(unicast(6, {CATENA_FRAME_LINK_REQ, 6, 1}));
  hal.hall_value = 1.0f;
  for (int i = 0; i < 50; ++i) tail.step(hal);
  CHECK(hal.sent_to(6, CATENA_FRAME_LINK_ACK) == 0);
  CHECK(hal.debug.behind == CATENA_NONE);
}

TEST_CASE("a docking robot is linked once the tail acknowledges, and then follows") {
  FakeHal hal;
  catena::Robot robot(5, hal.p);
  const float ref = hal.p.ir_reference_range_mm;
  // Robot 3 dead ahead at the standoff, and it reports robot 5 in the middle of its zone.
  bool requested = false;
  for (int i = 0; i < 200 && !requested; ++i) {
    hal.inbox.clear();
    hal.inbox.push_back(status_flash(3, 0.0f, hal.p.seek_standoff_mm, ref, 3, 1, 0, kSingle));
    hal.inbox.push_back(unicast(3, {CATENA_FRAME_GUIDE, 0}));
    hal.sent.clear();
    robot.step(hal);
    requested = hal.sent_to(3, CATENA_FRAME_LINK_REQ) > 0;
  }
  REQUIRE(requested);
  CHECK(hal.debug.state == CATENA_STATE_DOCK);
  CHECK(hal.forward() == doctest::Approx(hal.p.dock_speed_mm_s));

  hal.inbox.clear();
  hal.inbox.push_back(status_flash(3, 0.0f, hal.p.pitch_mm, ref, 3, 1, 0, CATENA_FLAG_HEAD));
  hal.inbox.push_back(unicast(3, {CATENA_FRAME_LINK_ACK, 0, 3, 255, 0}));
  robot.step(hal);
  CHECK(hal.debug.state == CATENA_STATE_FOLLOW);
  CHECK(hal.debug.ahead == 3);
  CHECK(hal.debug.index == 1);
  CHECK(hal.debug.chain_size == 2);
  CHECK(hal.debug.chain_id == 3);
  CHECK(hal.sent_to(3, CATENA_FRAME_LINK_UP) == 1);
}

TEST_CASE("a link whose frames stop is dropped after the link timeout") {
  FakeHal hal;
  catena::Robot robot(5, hal.p);
  const float ref = hal.p.ir_reference_range_mm;
  hal.inbox.push_back(status_flash(3, 0.0f, hal.p.pitch_mm, ref, 3, 1, 0, kSingle));
  hal.inbox.push_back(unicast(3, {CATENA_FRAME_LINK_ACK, 0, 3, 255, 0}));
  robot.step(hal);  // a tip that landed unannounced: robot 3 offers the link, robot 5 takes it
  REQUIRE(hal.debug.ahead == 3);
  hal.inbox.clear();
  const int steps = static_cast<int>(hal.p.link_timeout_s * hal.p.tick_hz) + 2;
  for (int i = 0; i < steps; ++i) robot.step(hal);
  CHECK(hal.debug.ahead == CATENA_NONE);
  CHECK(hal.debug.chain_size == 1);
}

TEST_CASE("a follower regulates the gap from the hall reading the robot ahead reports") {
  FakeHal hal;
  catena::Robot robot(5, hal.p);
  const float ref = hal.p.ir_reference_range_mm;
  auto step_with_hall = [&](uint8_t hall) {
    hal.inbox.clear();
    hal.inbox.push_back(status_flash(3, 0.0f, hal.p.pitch_mm, ref, 3, 1, 0, CATENA_FLAG_HEAD));
    hal.inbox.push_back(unicast(3, {CATENA_FRAME_LINK_ACK, 0, 3, hall, 0}));
    robot.step(hal);
  };
  step_with_hall(255);
  REQUIRE(hal.debug.state == CATENA_STATE_FOLLOW);
  CHECK(hal.forward() < 0.0f);  // pressed against the hub: ease off
  step_with_hall(60);
  CHECK(hal.forward() > 0.0f);  // falling behind: close up
  hal.ring = 0.5f;              // the robot ahead is to the left of the base heading
  step_with_hall(190);
  CHECK(hal.turn() > 0.0f);
}
