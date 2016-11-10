#ifndef ROSLOG_NODE_TREE
#define ROSLOG_NODE_TREE

#include <string>
#include <ros/names.h>

#include <ncurses.h>

namespace node_tree {

std::vector<std::string> splitNodeName(std::string name);

class Namespace {
public:
  std::vector<Namespace*> children;
  Namespace* parent = nullptr;

  std::string name;

  Namespace* getChildByName(std::string name);
  size_t getIndexOfChild(Namespace* child);
  std::string getFullName();
};

class Node : public Namespace {
public:
};

class Tree {
public:
  Tree();

  Namespace root;
  Namespace* selected;
  void update(std::vector<std::string> node_names);
  void debugPrint();
  void drawCurses(WINDOW* window);
  void moveSelection(int offset);
};

}

#endif //ROSLOG_NODE_TREE
