# https://leetcode.com/problems/container-with-most-water/


# You are given an integer array height of length n. There are n vertical lines drawn such that the two endpoints of the ith line are (i, 0) and (i, height[i]).

# Find two lines that together with the x-axis form a container, such that the container contains the most water.

# Return the maximum amount of water a container can store.

# Notice that you may not slant the container.

# Input: height = [1,8,6,2,5,4,8,3,7]
# Output: 49
# Explanation: The above vertical lines are represented by array [1,8,6,2,5,4,8,3,7]. In this case, the max area of water (blue section) the container can contain is 49.


class Solution:
    def maxArea(self, height: list[int]) -> int:
        max_water = 0
        for i, h1 in enumerate(height):
            for j, h2 in enumerate(height[i+1:]):

                bucket = height[i:i+j]
                if len(bucket) > 1:

                    wall_height = min([bucket[0], bucket[-1]])
                    width = j
                    area = wall_height * width

                    print(i, i+j, height[i:i+j], wall_height, area)

                    if area > max_water:
                        max_water = area
        return max_water

height = [1,8,6,2,5,4,8,3,7]
solution = Solution()
max_water = solution.maxArea(height)

print("max water: ", max_water)
