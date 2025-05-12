#include "joints_state.h"

void js_init(joints_state *js, int cnt_per_rev, int rate){
  js -> cntprev = cnt_per_rev;
  js -> rate = rate;
  js -> diffcnt = 0;
  // js -> total_cnt = 0;
  // js -> rad = 0.0;
  js -> radps = 0.0;
}

void update_joints_state(joints_state* js){
  // Calculate position
  // js -> total_cnt += js -> diffcnt;
  // js -> rad = js -> total_cnt * 2 * PI / js -> cntprev;

  // Calculate velocity
  int64_t cntps = js -> diffcnt * js -> rate;
  js -> radps = cntps * 2 * PI / js -> cntprev;
}