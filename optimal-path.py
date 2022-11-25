import osmnx as ox
import networkx as nx
import plotly.graph_objects as go
import numpy as np
import tkinter
import tkintermapview


destinations = []
start = None


def select_positions():
    """
    Função utiliza para escolher pontos no mapa a serem utilizados no caminho.

    O primeiro ponto selecionado é a origem e o restante são os pontos que demve ser visitados
    """
    def add_marker_event(coords):
        global start

        if start is None:
            start = coords
            new_marker = map_widget.set_marker(
                coords[0], coords[1], text="Start", marker_color_outside='#48a2f7', marker_color_circle='white')
        else:
            destinations.append(coords)
            new_marker = map_widget.set_marker(
                coords[0], coords[1], text="Local")

    # create tkinter window
    root_tk = tkinter.Tk()
    root_tk.geometry(f"{800}x{600}")
    root_tk.title("map_view_example.py")

    # create map widget
    map_widget = tkintermapview.TkinterMapView(
        root_tk, width=800, height=600, corner_radius=0)
    map_widget.place(relx=0.5, rely=0.5, anchor=tkinter.CENTER)

    map_widget.set_position(-23.319560313133028, -51.12428291797715)
    map_widget.set_zoom(17)

    map_widget.add_left_click_map_command(add_marker_event)

    root_tk.mainloop()


def plot_path(lat, long, origin_point, destination_points):
    """
    Given a list of latitudes and longitudes, origin 
    and destination point, plots a path on a map

    Parameters
    ----------
    lat, long: list of latitudes and longitudes
    origin_point, destination_point: co-ordinates of origin
    and destination
    Returns
    -------
    Nothing. Only shows the map.
    """
    # adding the lines joining the nodes
    fig = go.Figure(go.Scattermapbox(
        name="Path",
        mode="lines",
        lon=long,
        lat=lat,
        marker={'size': 10},
        line=dict(width=4.5, color='blue')))
    # adding source marker
    fig.add_trace(go.Scattermapbox(
        name="Source",
        mode="markers",
        lon=[origin_point[1]],
        lat=[origin_point[0]],
        marker={'size': 12, 'color': "red"}))

    for i in range(0, len(destination_points)):
        dest = destination_points[i]
        # adding destination marker
        fig.add_trace(go.Scattermapbox(
            name=f'Destination {i}',
            mode="markers",
            lon=[dest[1]],
            lat=[dest[0]],
            marker={'size': 12, 'color': 'green'}))

    # getting center for plots:
    lat_center = np.mean(lat)
    long_center = np.mean(long)
    # defining the layout using mapbox_style
    fig.update_layout(mapbox_style="stamen-terrain",
                      mapbox_center_lat=30, mapbox_center_lon=-80)
    fig.update_layout(margin={"r": 0, "t": 0, "l": 0, "b": 0},
                      mapbox={
                          'center': {'lat': lat_center,
                                     'lon': long_center},
                          'zoom': 13})
    fig.show()


def node_list_to_path(G, node_list):
    """
    Given a list of nodes, return a list of lines that together
    follow the path
    defined by the list of nodes.
    Parameters
    ----------
    G : networkx multidigraph
    route : list
        the route as a list of nodes
    Returns
    -------
    lines : list of lines given as pairs ( (x_start, y_start), 
    (x_stop, y_stop) )
    """
    edge_nodes = list(zip(node_list[:-1], node_list[1:]))
    lines = []
    for u, v in edge_nodes:
        # if there are parallel edges, select the shortest in length
        data = min(G.get_edge_data(u, v).values(),
                   key=lambda x: x['length'])
        # if it has a geometry attribute
        if 'geometry' in data:
            # add them to the list of lines to plot
            xs, ys = data['geometry'].xy
            lines.append(list(zip(xs, ys)))
        else:
            # if it doesn't have a geometry attribute,
            # then the edge is a straight line from node to node
            x1 = G.nodes[u]['x']
            y1 = G.nodes[u]['y']
            x2 = G.nodes[v]['x']
            y2 = G.nodes[v]['y']
            line = [(x1, y1), (x2, y2)]
            lines.append(line)

    return lines


def get_nearest_nodes(G, source, dest):
    """
    Retorna os nós mais próximos das localizações selecionadas
    """
    origin_node = ox.nearest_nodes(G, source[1], source[0])
    destination_node = ox.nearest_nodes(
        G, dest[1], dest[0])

    return origin_node, destination_node


def get_closer_dest_to_start(G):
    """
    Retorna o nó cujo caminho é o de menor custo à partir da origem
    """
    min_cost = 100000000000

    for dest in destinations:
        origin_node, destination_node = get_nearest_nodes(G, start, dest)

        cost = nx.shortest_path_length(G, origin_node, destination_node)
        if cost < min_cost:
            min_cost = cost
            origin_node_min_cost = origin_node
            dest_node_min_cost = destination_node
            near_dest = dest

    return near_dest, origin_node_min_cost, dest_node_min_cost


def make_heuristic(G):
    def heuristic(a, b):
        nodes = G.nodes(data=True)

        # recebendo os dados dos nós
        a_data = [item for item in nodes if item[0] == a][0]
        b_data = [item for item in nodes if item[0] == b][0]

        x1, y1 = a_data[1]['x'], a_data[1]['y']
        (x2, y2) = b_data[1]['x'], b_data[1]['y']

        return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    return heuristic


def find_optimal_path():
    """
    Procura o melhor caminho entre os pontos selecionados e o desenha no mapa
    """
    G = ox.graph_from_point(
        (start[0], start[1]), dist=5000, dist_type='bbox', network_type='drive')

    route = list()

    near_dest, origin_node, destination_node = get_closer_dest_to_start(
        G)
    route = nx.astar_path(G, origin_node,
                          destination_node, make_heuristic(G), weight='length')  # calculando para o primeiro nó

    source = near_dest
    for dest in destinations:  # encontra o caminho entre cada par de pontos

        if source == dest:
            continue

        origin_node, destination_node = get_nearest_nodes(G, source, dest)

        # Encontra o caminho ótimo e concatena com os anteriores
        route = route[:-1] + nx.astar_path(G, origin_node,
                                           destination_node, make_heuristic(G), weight='length')
        source = dest

    # # getting the list of coordinates from the path
    # # (which is a list of nodes)
    lines = node_list_to_path(G, route)
    long = []
    lat = []
    for i in range(len(lines)):
        z = list(lines[i])
        l1 = list(list(zip(*z))[0])
        l2 = list(list(zip(*z))[1])
        for j in range(len(l1)):
            long.append(l1[j])
            lat.append(l2[j])

    plot_path(lat, long, start, destinations)


select_positions()
print(f'\n\nPonto de origem selecionado: {start}')
print('\nDestinos que devem visitados: \n')
for dest in destinations:
    print(dest)
print()

find_optimal_path()
