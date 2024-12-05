# Given a string containing just the characters '(' and ')', find the length of the longest valid (well-formed) parentheses substring.

# Example 1:
# Input: s = "(()"
# Output: 2
# Explanation: The longest valid parentheses substring is "()".

# Example 2:
# Input: s = ")()())"
# Output: 4
# Explanation: The longest valid parentheses substring is "()()".

# Example 3:
# Input: s = ""
# Output: 0


class Solution:
    def longestValidParentheses(self, s: str) -> int:

        max_count = 0

        for i in range(len(s)):

            sub_s = s[i:]

            print("sub s: ", sub_s)

            if sub_s[0] == "(":
                counter = 0
                place = 0
                for c in sub_s:
                    place += 1

                    if c == "(":
                        counter += 1
                    if c == ")":
                        counter -= 1

                    print("counter: ", counter, ", place: ", place)

                    if counter == 0:
                        print("sub max: ", place)

                    if counter == 0 and place > max_count:
                        max_count = place

        return max_count


s = ")()())"
sol = Solution()
max_count = sol.longestValidParentheses(s)
print("max: ", max_count)
