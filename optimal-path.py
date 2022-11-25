from random import randrange
import osmnx as ox
import networkx as nx
import plotly.graph_objects as go
import numpy as np
import tkinter
import tkintermapview


locais = []
start = None


def select_positions():
    '''
    Função utiliza para escolher pontos no mapa a serem utilizados no caminho.

    O primeiro ponto selecionado é a origem e o restante são os pontos que demve ser visitados
    '''

    global root_tk

    def add_marker_event(coords):
        global start

        if start is None:
            start = coords
            new_marker = map_widget.set_marker(
                coords[0], coords[1], text="Start", marker_color_outside='#48a2f7', marker_color_circle='white')
        else:
            locais.append(coords)
            new_marker = map_widget.set_marker(
                coords[0], coords[1], text="Local")

    def destroy():
        root_tk.destroy()

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

    tkinter.Button(root_tk, text="Encerrar", command=destroy).pack()
    root_tk.mainloop()


def plot_path(lat, long, origin_point, destination_point):
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

    # adding destination marker
    fig.add_trace(go.Scattermapbox(
        name="Destination",
        mode="markers",
        lon=[destination_point[1]],
        lat=[destination_point[0]],
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


def find_optimal_path():
    '''
    Procura o melhor caminho entre os pontos selecionados e desenha o caminho no mapa
    '''

    G = ox.graph_from_point(
        (start[0], start[1]), dist=1000, dist_type='bbox', network_type='drive')
    # G = ox.graph_from_point(
    #     (-23.31943220339117, -51.1242936321194), dist=1000, dist_type='bbox', network_type='drive')
    # Plotting the map graph
    # ox.plot_graph(G)

    # nodes
    # nodes = list(G.nodes(data=True))
    # print(nodes[1], nodes[1][1]['x'])

    # define origin and desination locations
    # orig = nodes[randrange(0, len(nodes))][1]  # pegando um node aleatorio
    # start = (orig['x'], orig['y'])
    # print(start)

    # dest = nodes[randrange(0, len(nodes))][1]  # pegando um node aleatorio
    # destination = (dest['x'], dest['y'])
    # print(destination)
    destination = locais[0]  # pegando apenas o primeiro local

    # get the nearest nodes to the locations
    origin_node = ox.nearest_nodes(G, start[1], start[0])
    destination_node = ox.nearest_nodes(
        G, destination[1], destination[0])

    # printing the closest node id to origin and destination points
    # print(origin_node)
    # print(destination_node)

    # Finding the optimal path
    route = nx.shortest_path(G, origin_node, destination_node, weight='length')
    print(route)

    # getting the list of coordinates from the path
    # (which is a list of nodes)
    lines = node_list_to_path(G, route)
    long2 = []
    lat2 = []
    for i in range(len(lines)):
        z = list(lines[i])
        l1 = list(list(zip(*z))[0])
        l2 = list(list(zip(*z))[1])
        for j in range(len(l1)):
            long2.append(l1[j])
            lat2.append(l2[j])

    plot_path(lat2, long2, start, destination)


select_positions()
print(f'\n\nPonto de origem selecionado: {start}')
print(f'\nLocais que devem visitados: \n{locais}')

find_optimal_path()
