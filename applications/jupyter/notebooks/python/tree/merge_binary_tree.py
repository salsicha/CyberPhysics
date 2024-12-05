
# https://codinginterviewsmadesimple.substack.com/p/how-to-use-easy-leetcode-problems
# They can all be broken down as the following:
# def treeFunc(root, **other params):
#     if(root==null):
#         return baseCase/other relevant params
#     val=operation(treeFunc(root.left), treeFunc(root.right))
#     return val


# You are given two binary trees root1 and root2.
# Imagine that when you put one of them to cover the other, 
# some nodes of the two trees are overlapped while the others are not. 
# You need to merge the two trees into a new binary tree. 
# The merge rule is that if two nodes overlap, 
# then sum node values up as the new value of the merged node. 
# Otherwise, the NOT null node will be used as the node of the new tree.
# Return the merged tree.
# Note: The merging process must start from the root nodes of both trees.
#
# Example 1:
# (see image)
# Input: root1 = [1,3,2,5], root2 = [2,1,3,null,4,null,7]
# Output: [3,4,5,5,4,null,7]
#
# Example 2:
# Input: root1 = [1], root2 = [1,2]
# Output: [2,2]
#
# # Constraints:
#     The number of nodes in both trees is in the range [0, 2000].
#     -10^4 <= Node.val <= 10^4


def recur(index, one, two, output):

    while len(one) < len(two):
        one.append(None)
    while len(two) < len(one):
        two.append(None)

    if index >= len(one) and index >= len(two):
        return

    if one[index] and two[index]:
        sum = one[index] + two[index]
        output.append(sum)
    elif one[index]:
        output.append(one[index])
    elif two[index]:
        output.append(two[index])
    else:
        output.append(None)

    index += 1
    recur(index, one, two, output)


def solution(index, one, two, output):
    while len(one) < len(two):
        one.append(None)
    while len(two) < len(one):
        two.append(None)
    recur(index, one, two, output)


output = []
index = 0
root1 = [1, 3, 2, 5]
root2 = [2, 1, 3, None, 4, None, 7]

solution(index, root1, root2, output)

print(output)