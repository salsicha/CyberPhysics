
# Given an array of intervals where intervals[i] = [starti, endi], 
# merge all overlapping intervals, 
# and return an array of the non-overlapping intervals that cover all the intervals in the input.


intervals = [[1,3],[2,6],[8,10],[15,18]]

print(intervals)

output = []

for x, interval in enumerate(intervals):
    if len(output) == 0:
        output.append(interval)
    
    for y, out in enumerate(output):
        print(interval, "_", out)
        if interval[0] == out[0] and interval[1] == out[1]:
            print("pass")
        elif interval[0] <= out[0] and interval[1] >= out[0]:
            out[0] = interval[0]
            out[1] = max(interval[1], out[1])
        elif interval[0] <= out[1] and interval[1] >= out[1]:
            out[0] = min(interval[0], out[0])
            out[1] = interval[1]
        else:
            if interval not in output:
                output.append(interval)

print(output)