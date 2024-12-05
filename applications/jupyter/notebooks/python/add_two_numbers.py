
# You are given two non-empty linked lists representing two non-negative integers. 
# The digits are stored in reverse order, and each of their nodes contains a single digit. 
# Add the two numbers and return the sum as a linked list.
# You may assume the two numbers do not contain any leading zero, except the number 0 itself.


# Input: l1 = [2,4,3], l2 = [5,6,4]
# Output: [7,0,8]
# Explanation: 342 + 465 = 807.

# Input: l1 = [0], l2 = [0]
# Output: [0]


l1 = [2,4,3]
l2 = [5,6,4]

def switch(l):
    out_list = []
    l.reverse()
    for int_l in l:
        out_list.append(str(int_l))

    output = "".join(out_list)
    output = int(output)
    return output

int_1 = switch(l1)
int_2 = switch(l2)

sum_l = int_1 + int_2
out_s = str(sum_l)

output = []
for out in out_s:
    output.append(out)

# output = output[::-1]
output.reverse()

print(output)
