#include <ros/ros.h>

#include <rosgraph_msgs/Log.h>
#include <roscpp/SetLoggerLevel.h>

#include <iostream>
#include <deque>

#include <ncurses.h>

#include <roslog/node_tree.h>

enum class State {
  LOG,
  NODES
};

WINDOW* log_window;
std::deque<rosgraph_msgs::Log::ConstPtr> log_msgs;
size_t log_top_index = 0;
bool log_list_autoscroll = true;

State state = State::LOG;

node_tree::Tree nodeTree;

void init_colors() {
    start_color();
    use_default_colors();
    init_pair(1, COLOR_WHITE, -1); //-1 = default
    init_pair(2, COLOR_YELLOW, -1);
    init_pair(3, COLOR_RED, -1);
}

int get_color_from_level(uint8_t level) {
    if (level == 1) return 1;
    if (level == 2) return 1;
    if (level == 4) return 2;
    if (level == 8) return 3;
    if (level == 16) return 3;
    return 3;
}

std::string get_tag_from_level(uint8_t level) {
  if (level == 1) return "[DEBUG]";
  if (level == 2) return "[INFO]";
  if (level == 4) return "[WARN]";
  if (level == 8) return "[ERROR]";
  if (level == 16) return "[FATAL]";
  return "[UNKNOWN LEVEL]";
}

void log_draw() {
    if (state != State::LOG) return;

    werase(log_window);
    for (size_t line = 0; line < LINES; ++line) {
        size_t log_idx = log_top_index - line;
        if (log_idx < 0 || log_idx >= log_msgs.size()) {
          break;
        } else {
          wattron(log_window, COLOR_PAIR(get_color_from_level(log_msgs[log_idx]->level)));
          mvwaddstr(log_window, line, 0, (get_tag_from_level(log_msgs[log_idx]->level) + " " + log_msgs[log_idx]->msg).substr(0, COLS).c_str());
          wattroff(log_window, COLOR_PAIR(get_color_from_level(log_msgs[log_idx]->level)));
        }
    }
    wrefresh(log_window);
}

void nodes_list_draw() {
  if (state != State::NODES) return;

  werase(log_window);
  nodeTree.drawCurses(log_window);
  wrefresh(log_window);
}

void rosout_cb(rosgraph_msgs::Log::ConstPtr log_msg) {
    log_msgs.push_back(log_msg);
    if (log_list_autoscroll) {
      log_top_index = log_msgs.size() - 1;
    }
    log_draw();
}

std::string int_level_to_string(uint8_t level) {
  if (level == 1) return "debug";
  if (level == 2) return "info";
  if (level == 4) return "warn";
  if (level == 8) return "error";
  if (level == 16) return "fatal";
}

void set_logger_level(node_tree::TreeNode* ns, std::string level) {
  ns->doForeachChildNode([level](node_tree::Node* n){
    roscpp::SetLoggerLevel msg;
    msg.request.logger = "ros";
    msg.request.level = level;
    if (ros::service::call(n->getFullName() + "/set_logger_level", msg)) {

    } else {

    }
  });
}

void updateNodeTree(const ros::TimerEvent& event) {
  std::vector<std::string> nodeNames;
  ros::master::getNodes(nodeNames);
  nodeTree.update(nodeNames);
  nodes_list_draw();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "roslog", ros::init_options::AnonymousName);

    ros::NodeHandle nh;
    ros::Subscriber rosout_agg_sub = nh.subscribe("/rosout_agg", 100, rosout_cb);

    ros::Timer node_update_timer = nh.createTimer(ros::Duration(2.0), updateNodeTree);

    initscr();
    raw();
    keypad(stdscr, TRUE);
    noecho();
    curs_set(0); //Make cursor invisible
    
    init_colors();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    log_window = newwin(LINES, COLS, 0, 0);
    refresh();
    log_draw();
    nodes_list_draw();


    int ch;
    while (true) {
        ch = getch();
        if (ch == 'q' || ch == KEY_F(10)) {
            break;
        }
        if (state == State::LOG) {
          if (ch == 's') {
            log_list_autoscroll = !log_list_autoscroll;
          }
          if (!log_list_autoscroll) {
            if (ch == KEY_UP || ch == 'k') { //TODO Underflow on -1 on top of list
              log_top_index = std::min(log_msgs.size() - 1, log_top_index + 1);
            }
            if (ch == KEY_PPAGE) {
              log_top_index = std::min(log_msgs.size() - 1, log_top_index + LINES / 2);
            }
            if (ch == KEY_DOWN || ch == 'j') {
              log_top_index = std::max(size_t(0), log_top_index - 1);
            }
            if (ch == KEY_NPAGE) {
              log_top_index = std::max(size_t(0), log_top_index - LINES / 2);
            }
          }
        } else if (state == State::NODES) {
          if (ch == KEY_UP || ch == 'k') {
            nodeTree.moveSelection(-1);
          }
          if (ch == KEY_PPAGE) {
            nodeTree.moveSelection(-LINES / 2);
          }
          if (ch == KEY_DOWN || ch == 'j') {
            nodeTree.moveSelection(1);
          }
          if (ch == KEY_NPAGE) {
            nodeTree.moveSelection(LINES / 2);
          }
          if (ch >= '1' && ch <= '5') {
            uint8_t level = 1 << (ch - '1'); //1,2,4,8,16
            set_logger_level(nodeTree.selected, int_level_to_string(level));
          }
        }
        if (ch == 'n') {
          state = State::NODES;
        }
        if (ch == 'l') {
          state = State::LOG;
        }
        log_draw();
        nodes_list_draw();
    }

    spinner.stop();
    delwin(log_window);
    endwin();
    return 0;
}
