import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans

def kmeans_clustering_2d(n_clusters):
    # Set seed for reproducibility
    np.random.seed(42)

    # Rectangle dimensions
    rectangle_size = [500, 500]

    # Generate non-overlapping points
    x = np.random.uniform(low=1, high=499, size=50)
    y = np.random.uniform(low=1, high=499, size=50)

    # Combine x and y into a single array
    points = np.column_stack((x, y))

    # Apply k-means clustering
    kmeans = KMeans(n_clusters=n_clusters)
    kmeans.fit(points)

    # Get the centroids and labels for each point
    centroids = kmeans.cluster_centers_
    labels = kmeans.labels_

    # Plot the data points and centroids in 2D with green color for all data points
    plt.figure(figsize=(8, 6))
    for i in range(n_clusters):
        cluster_points = points[labels == i]
        if i == 0:
            plt.scatter(cluster_points[:, 0], cluster_points[:, 1], marker='o', color='blue', label=f'User locations')
        else:
            plt.scatter(cluster_points[:, 0], cluster_points[:, 1], marker='o', color='blue')

    plt.scatter(centroids[:, 0], centroids[:, 1], marker='x', color='red', label='Cluster centroids', s=60)

    # Add a single legend for both centroids and data points
    plt.legend(loc='upper right')

    plt.xlabel('Distance (meters)')
    plt.ylabel('Distance (meters)')
    plt.grid(True)
    plt.show()

    print("List of Centroid Coordinates:", centroids)
    for i in range(n_clusters):
        cluster_points = points[labels == i]
        print(f"Cluster {i + 1} Points:")
        for point in cluster_points:
            print(f'({point[0]:.2f}, {point[1]:.2f})')

    # Return the list of data points (x, y) and centroids
    return points, centroids

