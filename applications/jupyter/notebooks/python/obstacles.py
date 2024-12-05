
# For:
# box = [['#', '#', '.', '.', '.', '#', '.'],
#        ['#', '#', '#', '.', '.', '*', '.'],
#        ['#', '#', '#', '*', '.', '#', '.']]
# the output should be:
# solution(box) = [['#', '.', '.'],
#                  ['#', '.', '.'],
#                  ['#', '#', '.'],
#                  ['*', '#', '.'],
#                  ['.', '#', '#'],
#                  ['.', '*', '#'],
#                  ['#', '.', '#']]


# TODO: fix this


import numpy

def list_appending(hash_list, i):
    row_length = i
    hash_length = len(hash_list)
 
    list_diff = row_length - hash_length
    
    dot_list = []
    for x in range(list_diff):
        dot_list.append(".")
    
    output_list = dot_list + hash_list
    
    return output_list


def my_func(row):
  
    hash_list = []
        
    sub_list = []
        
    index_last_obstacle = 0
    for i, character in enumerate(row):
        if character == "#":
            hash_list.append("#")
        if character == "*":
            sub_list.append(list_appending(hash_list, i - index_last_obstacle))
            index_last_obstacle = i
            sub_list.append("*")
            hash_list = []
            
    sub_list.append(list_appending(hash_list, len(row) - index_last_obstacle))
    
    output = []
    for a_list in sub_list:
        output += a_list

    print(output)
    
    return output

def solution(box):
    output = []
    for row in box:
        output.append(my_func(row))
        
    arr = numpy.asarray(output)
    arr = numpy.rot90(arr, 3)
    
    print(arr.tolist())

    return arr.tolist()

box = [['#', '#', '.', '.', '.', '#', '.'],
       ['#', '#', '#', '.', '.', '*', '.'],
       ['#', '#', '#', '*', '.', '#', '.']]

solution(box)

