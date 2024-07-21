import pulp
import math
import itertools
import clusterCode as CC
k = 9
data_points,centroids = CC.kmeans_clustering_2d(k)
data_size =len(data_points)
print("data_size", data_size)
print("List of Centroid Coordinates:", centroids)
uav_coordinates = centroids
user_coordinates = data_points
I = range(len(uav_coordinates)) 
J = range(len(user_coordinates))  
M=1000000000
#M = 10000000000
alpha = 0.7
U = len(data_points)

beta = 200

Hmax = 1000
Hmin = 1 
D = [50] * U
#m_values = 1  
f = 2.4e9  # Replace with your actual frequency value
C_value = 3e8 
h0 = (Hmin + Hmax)/2 
Pmax = 5
Pmin = 1
PLmax = -289404150
theta = 45

prob = pulp.LpProblem("UAV_Optimization", pulp.LpMinimize)

x = pulp.LpVariable.dicts("x", [(i, j) for i in I for j in J], cat='Binary')    
h = pulp.LpVariable.dicts("h", [(i) for i in I], lowBound=Hmin, upBound=Hmax)  
t = pulp.LpVariable.dicts("t", [(i, j) for i in I for j in J], lowBound=0)  
L1 = pulp.LpVariable.dicts("L1", [(i, j) for i in I for j in J]) 
m = pulp.LpVariable.dicts("m", [(i) for i in I], cat='Binary')  
P = pulp.LpVariable.dicts("P", I, lowBound=Pmin, upBound=Pmax)  
y = pulp.LpVariable.dicts("y", [(i, j) for i in I for j in I if i != j], cat='Binary')
C = pulp.LpVariable.dicts("C", [(i, k) for i in I for k in I if i != k], cat='Binary')
S = pulp.LpVariable.dicts("S", [(i, k) for i in I for k in I if i != k], cat='Binary')
L2 = pulp.LpVariable.dicts("L2", [(i, k) for i in I for k in I]) 
 
#Objective
prob += pulp.lpSum(L1[i, j] for i in I for j in J)
#prob += pulp.lpSum(m[i] for i in I),"Objective"
#1: i is UAV, j is user
for j in J:
    prob += pulp.lpSum(x[i, j] for i in I) <= 1
#2
for i in I:
    for j in J:
        prob += x[i, j] <= m[i]

#3
prob += pulp.lpSum(m[i] for i in I) == P
#4
prob += pulp.lpSum(x[i, j] for i in I for j in J) >= alpha * U
#5
for i in I:
    prob += pulp.lpSum(D[j] * x[i, j] for j in J) <= beta * m[i]
#6
for i in I:
    prob += h[i] <= Hmax * m[i]

for i in I:
    prob += h[i] >= Hmin * m[i]
#7

# Constraints
for i in I:
    for j in J:
        dij_value = ((uav_coordinates[i][0] - user_coordinates[j][0]) ** 2 +
                     (uav_coordinates[i][1] - user_coordinates[j][1]) ** 2) ** 0.5
        aij_value = ((PLmax - (4 * 3.14 * f / C_value) ** 2 * (dij_value ** 2 - h0 ** 2)) /
                                   (4 * 3.14 * f / C_value) ** 2 * 2 * h0)
        prob += x[i, j] <= (M - h[i]) / (M - aij_value + 1/2)

#9
for i in I:
    for j in J:
        prob += L1[i, j] <= M*x[i, j]

#10
for i in I:
    for j in J:
        dij_value = ((uav_coordinates[i][0] - user_coordinates[j][0]) ** 2 +
                     (uav_coordinates[i][1] - user_coordinates[j][1]) ** 2) ** 0.5
        prob += t[i, j] <= h[i]
        prob += t[i, j] <= Hmax * x[i, j]
        prob += t[i, j] >= h[i] - (1 - x[i, j]) * Hmax
        L1[i, j] >= ((4 * 3.14 * f * C_value)**2 * dij_value**2 - ((2 * Hmin + Hmax) / 2)**2) * x[i, j] + (4 * 3.14 * f * C_value)**2 * 2 * ((Hmin + Hmax) / 2) * t[i, j]
        prob += L1[i, j]

#11
for i in I:
    for j in J:
        dij_value = ((uav_coordinates[i][0] - user_coordinates[j][0]) ** 2 +
                     (uav_coordinates[i][1] - user_coordinates[j][1]) ** 2) ** 0.5
        prob += (1/math.tan(theta) * x[i, j]*dij_value) <= h[i] 

   
prob.solve()
print("Status:", pulp.LpStatus[prob.status])

if pulp.LpStatus[prob.status] == "Optimal":
    optimal_solution = {}
    for v in prob.variables():
        optimal_solution[v.name] = v.varValue
    optimal_objective_value = pulp.value(prob.objective)

    print("Optimal Solution:")
else:
    print("Sorry")   
# Assuming that the problem has been solved successfully
if pulp.LpStatus[prob.status] == "Optimal":
    # Print the points for which m[i] = 1 and the corresponding h values
    print("Selected points:")

for points in uav_coordinates:    
    if pulp.value(m[i]) == 1:
        print(f"Point {i}: h = {pulp.value(h[i])}")
        print(f"Point {i}: {uav_coordinates[i]}") 
    else:
        print("Optimization problem not solved successfully.")


