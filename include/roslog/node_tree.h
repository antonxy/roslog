#ifndef ROSLOG_NODE_TREE
#define ROSLOG_NODE_TREE

#include <string>
#include <functional>

#include <ros/names.h>

#include <ncurses.h>

namespace node_tree {

std::vector<std::string> splitNodeName(std::string name);

class Node;
class Namespace;

class TreeNode {
public:
  virtual ~TreeNode() {}
  Namespace* parent = nullptr;

  std::string name;
  std::string getFullName();
  void doForeachChildNode(std::function<void(Node*)> fun);
};

class Namespace : public TreeNode {
public:

  std::vector<TreeNode*> children;

  template<class T>
  T* getChildByNameAndType(std::string name) {
    for (TreeNode* child : children) {
      T* t_child = dynamic_cast<T*>(child);
      if (t_child && t_child->name == name) {
        return t_child;
      }
    }
    return nullptr;
  }

  size_t getIndexOfChild(TreeNode* child);
};

class Node : public TreeNode {
public:
};

class Tree {
public:
  Tree();

  Namespace root;
  TreeNode* selected;
  void update(std::vector<std::string> node_names);
  void debugPrint();
  void drawCurses(WINDOW* window);
  void moveSelection(int offset);
};

}

#endif //ROSLOG_NODE_TREE
