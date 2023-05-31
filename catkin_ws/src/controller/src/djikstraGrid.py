#!/usr/bin/env python3
import heapq

def shortest_path(occupancy_grid, start, end):
    # Crea una matriz para almacenar la distancia desde el inicio a cada posición.
    distances = [[float('inf')] * len(occupancy_grid[0]) for _ in range(len(occupancy_grid))]
    # Crea una matriz para almacenar el nodo anterior (para reconstruir el camino).
    previous_nodes = [[None] * len(occupancy_grid[0]) for _ in range(len(occupancy_grid))]

    # La distancia desde el inicio hasta sí mismo es 0.
    distances[start[0]][start[1]] = 0

    # Crea una cola de prioridad y añade la posición de inicio con una prioridad de 0.
    queue = [(0, start)]

    while queue:
        # Obtiene la posición con la menor prioridad (distancia) y la elimina de la cola.
        current_distance, current_position = heapq.heappop(queue)

        # Si esta posición ya tiene una distancia menor (ha sido visitada con una ruta más corta), omítela.
        if current_distance > distances[current_position[0]][current_position[1]]:
            continue

        # Comprueba todas las posiciones vecinas.
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            next_position = (current_position[0] + dx, current_position[1] + dy)

            # Comprueba si la próxima posición está dentro del mapa de ocupación y es accesible.
            if (0 <= next_position[0] < len(occupancy_grid) and
                0 <= next_position[1] < len(occupancy_grid[0]) and
                occupancy_grid[next_position[0]][next_position[1]] != 100):
                # Calcula la distancia desde el inicio hasta esta posición.
                next_distance = current_distance + 1  # Asume que el costo para moverse a una celda adyacente es 1.

                # Si esta es una ruta más corta hasta esta posición, actualiza la matriz de distancias y añade la posición a la cola.
                if next_distance < distances[next_position[0]][next_position[1]]:
                    distances[next_position[0]][next_position[1]] = next_distance
                    previous_nodes[next_position[0]][next_position[1]] = current_position
                    heapq.heappush(queue, (next_distance, next_position))

    # Si no se puede llegar al final, devuelve None.
    if previous_nodes[end[0]][end[1]] is None:
        return None

    # Reconstruye el camino desde el final hasta el inicio.
    path = []
    while end is not None:
        path.append(end)
        end = previous_nodes[end[0]][end[1]]
    path.reverse()  # Invierte el camino para que vaya desde el inicio hasta el final.

    return path
