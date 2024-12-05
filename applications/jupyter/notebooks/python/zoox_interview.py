'''
**Q1** 

We have an array of elements. Each element has a unique integer id and a set of 64 flags represented by a single 64-bit integer named bit_flags. For example, an element has bit_flags = 5, meaning the element has flag 0 and flag 2.

Implement a function: given a list of such elements and a flag number, the function should find the last element that has the given flag number, and return the id of that element.

Example: 

Input:

list = [

{id: 0, bit_flags: 5}, // 000..00101 -> flag 0 and flag 2

{id: 1, bit_flags: 3}, // 000…00011

{id: 2, bit_flags: 10}, // 000…01010

{id: 3, bit_flags: 4} // 000…00100

]

flag = 1

Output: ?
''' 

list = [

{id: 0, bit_flags: 5}, # 000..00101 -> flag 0 and flag 2

{id: 1, bit_flags: 3}, # 000…00011 

{id: 2, bit_flags: 10}, # 000…01010

{id: 3, bit_flags: 4} # 000…00100

]

flag = 1 # 010

last = 0

for place, record in enumerate(list):
    bit_flag = record["bit_flags"]

    out = bit_flag >> flag & 1

    if out == True:
        last = place

print(last)

'''
**Q2**

You're given an array of numbers like [5, 4, 9, 5, 6, 7], representing stock price ordered by time. Compute the maximum profit one could have made by buying and then selling one share in that time range. 

NOTE: 

- Buy and sell once
- Must buy before sell
- No negative profit; min should be 0
''' 

input = [14, 20, 4, 9, 5, 6, 7]

max_val = 0

max_diff = 0

for i, val in enumerate(input[::-1]):
    if val > max_val:
        max_val = val

    diff = max_val - input[len(input)-1-i]

    if diff > max_diff:
        max_diff = diff

'''
**Q3** 

You are given a list of unique positive integers. Determine if numbers from the list can be plugged into Y=MX+B such that the equation is correct, and output the values if a solution exists. It is permissible to reuse numbers from the list. 

Example: 

1. [2,3,4,9,10], output 10=2*3+4 OR 10=2*4+2
''' 

input = [2,3,4,9,10]

input_len = len(input)

output = []

for i in range(input_len):
    for j in range(input_len):
        for k in range(input_len):
            for l in range(input_len):

                y = input[i]
                m = input[j]
                x = input[k]
                b = input[l]

                if y == m*x+b:
                    output = [y, m, x, b]

print(output)


slope = []

for i in range(input_len):
    for j in range(input_len):
        slope.append(input[i] * input[j])

slope_and_offset = []

for i in range(len(slope)):
    for j in range(input_len):
        slope_and_offset.append(slope[i] + input[j])


for i in range(input_len):
    y = input[i]
    for j in range(len(slope_and_offset)):
        if y == slope_and_offset[j]
            
