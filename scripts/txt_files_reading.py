# Python code to
# demonstrate readlines()

L = ["Geeks\n", "for\n", "Geeks\n"]

# writing to file
file1 = open('/home/ramanb/gps_ws/src/husky_gps/way_points/gps_from_teleop.txt', 'w')

file1.writelines(L)

for i in range(10):

    file1.writelines(L)
file1.close()

# # Using readlines()
# file1 = open('/home/ramanb/gps_ws/src/husky_gps/way_points/points_sim.txt', 'r')
# Lines = file1.readlines()
# print(len(Lines))
# count = 0
# # Strips the newline character
# for line in Lines:
#     count += 1
#     # print("Line{}: {}".format(count, line.strip()))
#     print(type(line))
#     print(line.strip()[0])
#     l= line.strip()
#     print(l.split())

# x = l.split(", ")
# split