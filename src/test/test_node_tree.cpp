#include <gtest/gtest.h>

#include <roslog/node_tree.h>

TEST(NodeTree, insert) {
  node_tree::Tree tree;
  std::vector<std::string> nodes;
  nodes.push_back("/foo/bar/baz");
  nodes.push_back("/foo/bar/baa");
  nodes.push_back("/foo/baz");
  nodes.push_back("/foo/foo/foo");
  nodes.push_back("/foo/bar");
  tree.update(nodes);

  tree.debugPrint();

  ASSERT_EQ("bar" , tree.root.getChildByName("foo")->getChildByName("bar")->name);
}


int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
