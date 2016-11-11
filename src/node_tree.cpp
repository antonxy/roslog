#include <roslog/node_tree.h>

#include <sstream>
#include <iostream>

namespace {
void split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss;
    ss.str(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
}


std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}
}

node_tree::Tree::Tree()
{
  selected = &root;
}

void node_tree::Tree::update(std::vector<std::string> node_names)
{
  //Insert Nodes not yet in list
  for (std::string name : node_names) {
    std::vector<std::string> name_parts = splitNodeName(name);

    Namespace* current_namespace = &root;
    for (size_t i = 0; i < name_parts.size() - 1; ++i) {
      Namespace* child = current_namespace->getChildByNameAndType<Namespace>(name_parts[i]);
      if (child == nullptr) {
        child = new Namespace;
        child->name = name_parts[i];
        child->parent = current_namespace;
        current_namespace->children.push_back(child);
      }
      current_namespace = child;
    }

    Node* node = current_namespace->getChildByNameAndType<Node>(name_parts.back());
    if (node == nullptr) {
      node = new Node;
      node->name = name_parts.back();
      node->parent = current_namespace;
      current_namespace->children.push_back(node);
    }

  }

  //Remove nodes not seen again

}

namespace {
void debugPrintN(node_tree::TreeNode* node, unsigned int level) {
  for (unsigned int i = 0; i < level; ++i) {
    std::cout << "  ";
  }
  std::cout << node->name << std::endl;
  if (node_tree::Namespace* ns = dynamic_cast<node_tree::Namespace*>(node)) {
    for (node_tree::TreeNode* child : ns->children) {
      debugPrintN(child, level + 1);
    }
  }
}
}

void node_tree::Tree::debugPrint()
{
  debugPrintN(&root, 0);
}

namespace {
unsigned int drawCursesN(WINDOW* window, node_tree::Tree* tree, node_tree::TreeNode* t, unsigned int level, unsigned int line) {
  if (line < LINES) {
    if (t == tree->selected) {
      wattron(window, A_REVERSE);
    }
    std::string addon = "  ";
    if (node_tree::Node* node = dynamic_cast<node_tree::Node*>(t)) {
      addon = " (N)";
    }
    mvwaddstr(window, line, level, (t->name + addon).c_str());
    wattroff(window, A_REVERSE);
  }
  unsigned int linesDrawn = 1;
  if (node_tree::Namespace* ns = dynamic_cast<node_tree::Namespace*>(t)) {
    for (node_tree::TreeNode* sub_ns : ns->children) {
      linesDrawn += drawCursesN(window, tree, sub_ns, level + 1, line + linesDrawn);
    }
  }
  return linesDrawn;
}
}

void node_tree::Tree::drawCurses(WINDOW *window)
{
  drawCursesN(window, this, &root, 0, 0);
}

void node_tree::Tree::moveSelection(int offset)
{
  while (offset > 0) {
    Namespace* selected_ns = dynamic_cast<Namespace*>(selected);
    if (selected_ns && selected_ns->children.size() > 0) {
      selected = selected_ns->children[0];
    } else {
      //selected -> parent -> child
      Namespace* parent = selected->parent;
      TreeNode* me = selected;
      while (true) {
        size_t my_idx = parent->getIndexOfChild(me);
        if (my_idx < parent->children.size() - 1) {
          selected = parent->children[my_idx + 1];
          break; // Found next element
        } else {
          me = parent;
          parent = parent->parent;
          if (parent == nullptr) {
            break; // End of tree
          }
        }
      }
    }
    offset--;
  }

  while (offset < 0) {
    if (selected->parent == nullptr) {
      break; // We are at the root
    }

    size_t my_idx = selected->parent->getIndexOfChild(selected);
    if (my_idx > 0) {
      //selected -> parent -> child
      TreeNode* sibling_above = selected->parent->children[my_idx - 1];
      //Find last transitive child of sibling_above
      TreeNode* node_above = sibling_above;
      while(true) {
        Namespace* ns_above = dynamic_cast<Namespace*>(node_above);
        if (ns_above && ns_above->children.size() > 0) {
          node_above = ns_above->children.back();
        } else {
          break;
        }
      }
      selected = node_above;
    } else {
      selected = selected->parent;
    }
    offset++;
  }
}


std::vector<std::string> node_tree::splitNodeName(std::string name)
{
  std::string clean_name = ros::names::clean(name);
  if (clean_name[0] != '/') {
    ROS_FATAL_STREAM("Node name " << clean_name << " is not absolute");
  }
  return split(clean_name.substr(1), '/');
}

size_t node_tree::Namespace::getIndexOfChild(node_tree::TreeNode *child)
{
  for (size_t i = 0; i < children.size(); ++i) {
    if (children[i] == child) {
      return i;
    }
  }

  return std::numeric_limits<size_t>::max();
}

std::string node_tree::TreeNode::getFullName()
{
  std::string name = "";
  TreeNode* ns = this;
  while(true) {
    name = ns->name + "/" + name;
    ns = ns->parent;
    if (ns == nullptr) {
      break;
    }
  }
  return name;
}

void node_tree::TreeNode::doForeachChildNode(std::function<void (node_tree::Node *)> fun)
{
  if (Node* node = dynamic_cast<Node*>(this)) {
    fun(node);
  } else if (Namespace* ns = dynamic_cast<Namespace*>(this)) {
    for (TreeNode* child : ns->children) {
      child->doForeachChildNode(fun);
    }
  } else {
    ROS_FATAL("WUT? treenode is neither node nor namespace?");
  }
}
