import pulp
import math

uav_coordinates = [(20,30),(40,20),(10,10),(45,45),(35,35),(20,50)] 
user_coordinates = [(8,8),(12,8),(9,15),(12,15),(18,28),(22,28),(18,30),(24,30),(19,35),(22,35),(38,18),(42,18),(38,26),(42,26),(99,25)] 

I = range(len(uav_coordinates)) 
J = range(len(user_coordinates))  
M = 1000
alpha = 1  
U = 15
R=50
beta = [20,30,40,35,35,10] 
Pmax = 7
Pmin = 1 
Hmax = 500
Hmin = 100
D = [2,2,2,2,2,2,2,2,2,2,2,2,2,2,2]
#m_values = 1  
f = 2*10**9
C = 3*10**8  
h0 = (Hmin + Hmax)/2 

PLmax = 50000000
theta = 45

dij = {}
for i in I:
    for j in J:
        dij[i, j] = math.sqrt((uav_coordinates[i][0] - user_coordinates[j][0])**2 + (uav_coordinates[i][1] - user_coordinates[j][1])**2)

aij = {}
for i in I:
    for j in J:
        aij[i,j] = ((PLmax - (4 * 3.14 * f / C)**2 * (dij[i, j]**2 - h0**2)) / ((4 * 3.14 * f / C)**2 * 2 * h0))

prob = pulp.LpProblem("UAV_Optimization", pulp.LpMinimize)


x = pulp.LpVariable.dicts("x", [(i, j) for i in I for j in J], cat='Binary')  
h = pulp.LpVariable.dicts("h", [(i) for i in I], lowBound=Hmin)  
t = pulp.LpVariable.dicts("t", [(i, j) for i in I for j in J], lowBound=0)  
k = pulp.LpVariable.dicts("k", [(i, j) for i in I for j in J], lowBound=0) 
m = pulp.LpVariable.dicts("m", I, cat='Binary')  
P = pulp.LpVariable("P", lowBound=1)  
y = pulp.LpVariable.dicts("y", [(i, j) for i in I for j in I if i != j], cat='Binary')

prob += pulp.lpSum(k[i, j] for i in I for j in J)

for j in J:
    prob += pulp.lpSum(x[i, j] for i in I) <= 1

for i in I:
    for j in J:
        prob += x[i, j] <= m[i]

prob += P >= Pmin
prob += P <= Pmax
prob += pulp.lpSum(m[i] for i in I) == P

prob += pulp.lpSum(x[i, j] for i in I for j in J) >= alpha * U

for i in I:
    prob += pulp.lpSum(D[j] * x[i, j] for j in J) <= beta[i] * m[i]

prob += pulp.lpSum(m[i] for i in I) == P

for i in I:
    prob += h[i] <= Hmax * m[i]

for i in I:
    prob += h[i] >= Hmin * m[i]

for i in I:
    for j in J:
        prob += (1/math.tan(theta) * x[i, j]*dij[i, j]) <= h[i] 
#
for i in I:
    for j in J:
        prob += x[i, j] <= (M - h[i]) / (M - aij[i,j] + 1/2)

for i in I:
    for j in J:
        k[i, j] >= ((4 * 3.14 * f * C)**2 * dij[i, j]**2 - ((2 * Hmin + Hmax) / 2)**2) * x[i, j] + (4 * 3.14 * f * C)**2 * 2 * ((Hmin + Hmax) / 2) * t[i, j]
        prob += k[i, j]

for i in I:
    for j in J:
        prob += t[i, j] <= h[i]
        prob += t[i, j] <= Hmax * x[i, j]
        prob += t[i, j] >= h[i] - (1 - x[i, j]) * Hmax
        #prob += k[i, j] >= ((4 * 3.14 * f * C)**2 * dij[i, j]**2 - ((2 * Hmin + Hmax) / 2)**2) * x[i, j] + (4 * 3.14 * f * C)**2 * 2 * ((Hmin + Hmax) / 2) * t[i, j]

for i in I:
    prob += pulp.lpSum(y[i, j] for j in I if i != j) >= 1  # Each UAV must have at least one neighbor


for i in I:
    for j in I:
        if i != j:
            distance_ij = math.sqrt((uav_coordinates[i][0] - uav_coordinates[j][0])**2 + (uav_coordinates[i][1] - uav_coordinates[j][1])**2)
            prob += y[i, j] <= x[i, j]  # If there is a link, both UAVs must be selected
            prob += y[i, j] <= (distance_ij <= R)              
prob.solve()
users_covered_count = [0] * len(uav_coordinates)

# Update users_covered_count based on the condition that reduced path loss should be less than PLmax
for i in I:
    for j in J:
        if pulp.value(x[i, j]) == 1 and aij[i, j] < PLmax:
            users_covered_count[i] += 1

# Sort UAV points by the number of users covered in descending order
sorted_uavs = sorted(enumerate(users_covered_count), key=lambda x: x[1], reverse=True)

# Select UAV points until 70% of users are covered
target_coverage = 0.7 * len(user_coordinates)
current_coverage = 0
selected_uavs = []

for i, users_covered in sorted_uavs:
    if current_coverage < target_coverage:
        selected_uavs.append(i)
        current_coverage += users_covered
    else:
        break

# Print the selected points
print("Selected Points:")
for i in selected_uavs:
    print(f"Point {i}: {uav_coordinates[i]} (Covering {users_covered_count[i]} users)")
