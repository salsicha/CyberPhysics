#
# Given an array of integers heights representing the histogram's bar height where the width of each bar is 1, return the area of the largest rectangle in the histogram.

# Example 1:
# Input: heights = [2,1,5,6,2,3]
# Output: 10
# Explanation: The above is a histogram where width of each bar is 1.
# The largest rectangle has an area = 10 units.

# Example 2:
# Input: heights = [2,4]
# Output: 4

# Example 3:
# Input: heights = [6,2,5,4,5,1,6]
# Output: 12

# Constraints:
#    1 <= heights.length <= 10^5
#    0<= heights[i] <= 10^4


heights = [6,2,5,4,5,1,6]

def max_area(heights):
    max_area = 0

    for i, x in enumerate(heights):
        width = 0

        for j, height_j in enumerate(heights[i:]):
            if height_j >= x:
                width += 1
            else:
                break

        for k, height_k in enumerate(heights[:i][::-1]):
            if height_k >= x:
                width += 1
            else:
                break

        area = width * x

        if area > max_area:
            max_area = area

    return max_area


max = max_area(heights)
print("max area: ", max)

