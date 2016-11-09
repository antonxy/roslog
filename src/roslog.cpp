#include <ros/ros.h>

#include <rosgraph_msgs/Log.h>

#include <iostream>
#include <deque>

#include <ncurses.h>

WINDOW* nodes_window;
std::vector<std::string> node_names;
size_t nodes_list_selected_index = 0;

WINDOW* log_window;
std::deque<rosgraph_msgs::Log::ConstPtr> log_msgs;
size_t log_top_index = 0;

void init_colors() {
    start_color();
    use_default_colors();
    init_pair(1, COLOR_WHITE, -1);
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

void nodes_list_draw() {
    wborder(nodes_window, ' ', '|', ' ', ' ', ' ', '|', ' ', '|');
    for (size_t i = 0; i < node_names.size(); ++i) {
        if (i >= 0 && i < LINES) {
            if (i == nodes_list_selected_index) wattron(nodes_window, A_REVERSE);
            mvwaddstr(nodes_window, i, 0, node_names[i].substr(0, 19).c_str());
            wattroff(nodes_window, A_REVERSE);
        }
    }
    wrefresh(nodes_window);
}

void log_draw() {
    for (size_t line = 0; line < LINES; ++line) {
        size_t log_idx = log_top_index - line;
        if (log_idx < 0 || log_idx >= log_msgs.size()) break;
        wattron(log_window, COLOR_PAIR(get_color_from_level(log_msgs[log_idx]->level)));
        mvwaddstr(log_window, line, 0, log_msgs[log_idx]->msg.substr(0, COLS - 20).c_str());
        wattroff(log_window, COLOR_PAIR(get_color_from_level(log_msgs[log_idx]->level)));
    }
    wrefresh(log_window);
}

void rosout_cb(rosgraph_msgs::Log::ConstPtr log_msg) {
    log_msgs.push_back(log_msg);
    log_top_index = log_msgs.size() - 1;
    log_draw();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "roslog", ros::init_options::AnonymousName);

    ros::NodeHandle nh;
    ros::Subscriber rosout_agg_sub = nh.subscribe("/rosout_agg", 100, rosout_cb);

    ros::master::getNodes(node_names);

    initscr();
    raw();
    keypad(stdscr, TRUE);
    noecho();
    curs_set(0); //Make cursor invisible
    
    init_colors();

    ros::AsyncSpinner spinner(1);
    spinner.start();

    nodes_window = newwin(LINES, 20, 0, 0);
    log_window = newwin(LINES, COLS - 20, 0, 20);
    refresh();
    nodes_list_draw();
    log_draw();


    int ch;
    while (true) {
        ch = getch();
        if (ch == 'q' || ch == KEY_F(10)) {
            break;
        }
        if (ch == KEY_UP) {
            --nodes_list_selected_index;
            if (nodes_list_selected_index < 0) nodes_list_selected_index = 0;
        }
        if (ch == KEY_DOWN) {
            ++nodes_list_selected_index;
            if (nodes_list_selected_index > node_names.size() - 1) nodes_list_selected_index = node_names.size() - 1;
        }
        nodes_list_draw();
    }

    delwin(nodes_window);
    endwin();
    spinner.stop();
    return 0;
}
