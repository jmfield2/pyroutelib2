#!/usr/bin/python
#----------------------------------------------------------------
# Routing for OSM data
#
#------------------------------------------------------
# Usage as library:
#   datastore = loadOsm('transport type')
#   router = Router(datastore)
#   result, route = router.doRoute(node1, node2)
#
# (where transport is cycle, foot, car, etc...)
#------------------------------------------------------
# Copyright 2007-2008, Oliver White
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#------------------------------------------------------
import sys
import math 
try:
  from .loadOsm import *
except (ImportError, SystemError):
  from loadOsm import *

import networkx as nx

class Router:
  def __init__(self, data):
    self.data = data
  def distance(self,n1,n2):
    """Calculate distance between two nodes"""
    lat1 = float(self.data.rnodes[n1][0]) * math.pi/180
    lon1 = float(self.data.rnodes[n1][1]) * math.pi/180
    lat2 = float(self.data.rnodes[n2][0]) * math.pi/180
    lon2 = float(self.data.rnodes[n2][1]) * math.pi/180
    # TODO: projection issues
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    dist2 = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * (math.sin(dlon/2))**2
    dist2 = 2 * math.asin( math.sqrt(dist2) )
    dist = 6371000 * dist2 # meters
    return(dist)
  
  def doRoute(self,start,end):
    """Do the routing"""
    self.searchEnd = end
    closed = [start]
    self.queue = []

    try: 
    	#rt = nx.dijkstra_path(self.data.G, start, end)
    	rt = nx.astar_path(self.data.G, start, end)
    except Exception as m:
        return ('no_route', [])
   
    #d = {'distance':nx.dijkstra_path_length(self.data.G, start, end), 'nodes': rt}
    d = {'distance':nx.astar_path_length(self.data.G, start, end), 'nodes': rt} 
    return ('success', [rt, d]) 
 
    # Start by queueing all outbound links from the start node
    blankQueueItem = {'end':-1,'distance':0,'nodes':str(start)}

    try:
      for i, weight in self.data.routing[start].items():
        n=self.data.findNode(float(self.data.rnodes[i][0]), float(self.data.rnodes[i][1]))
        self.addToQueue(start,i, blankQueueItem, weight)
    except KeyError:
      return('no_such_node',[])

    # Should only find local nodes to search instead of the entire graph XXX
   
    # Limit for how long it will search
    count = 0
    success = []
    while count < 5000:
      count = count + 1
      try:
        nextItem = self.queue.pop(0)
      except IndexError:
        # Queue is empty: failed
	if len(success) > 0: break # we could have found one path previously...
	#else: TODO: return partial route?
        else: return('no_route',[])
      x = nextItem['end']
      if x in closed:
        continue
      if x == end:
        # Found the end node - success
        routeNodes = [int(i) for i in nextItem['nodes'].split(",")]
        success.append( [routeNodes, nextItem] ) #return('success', [routeNodes, nextItem] )
        continue
      else: closed.append(x)
      try:
        for i, weight in self.data.routing[x].items():
          if not i in closed:
            self.addToQueue(x,i,nextItem, weight)
            #print x
 
      except KeyError as e:
        pass
    
    dist = [sys.maxint, 0]
    for x in success:
      if x[1]['distance'] < dist[0]: dist = [x[1]['distance'], x]

    #print success
 
    if len(success) <= 0: 
	for q in self.queue:
		n = []
		nodes = q['nodes'].split(',')
		for x in nodes:
			n.append( (float(self.data.rnodes[x][0]), float(self.data.rnodes[x][1])) )

                from polyline.codec import PolylineCodec
		#print PolylineCodec().encode(n)
	
	return ('gave_up', [])
    else: return ('success', dist[1])
  
  def addToQueue(self,start,end, queueSoFar, weight = 1):
    """Add another potential route to the queue"""

    # getArea() checks that map data is available around the end-point,
    # and downloads it if necessary
    #
    # TODO: we could reduce downloads even more by only getting data around
    # the tip of the route, rather than around all nodes linked from the tip
    if end not in self.data.rnodes: return
    end_pos = self.data.rnodes[end]
    #self.data.getArea(end_pos[0], end_pos[1])
    
    # If already in queue, ignore
    for test in self.queue:
      if test['end'] == end:
        return
    distance = self.distance(start, end)
    if(weight == 0):
      pass#return
    distance = distance / weight

    # Create a hash for all the route's attributes
    distanceSoFar = queueSoFar['distance']
    queueItem = { \
      'distance': distanceSoFar + distance,
      'maxdistance': distanceSoFar + self.distance(end, self.searchEnd),
      'nodes': queueSoFar['nodes'] + "," + str(end),
      'end': end}
    
    # Try to insert, keeping the queue ordered by decreasing worst-case distance
    count = 0
    for test in self.queue:
      if test['maxdistance'] > queueItem['maxdistance']:
        self.queue.insert(count,queueItem)
        break
      count = count + 1
    else:
      self.queue.append(queueItem)

if __name__ == "__main__":
  # Test suite - do a little bit of easy routing in birmingham
  data = LoadOsm("cycle")

  node1 = data.findNode(52.552394,-1.818763)
  node2 = data.findNode(52.563368,-1.818291)

  print(node1)
  print(node2)

  router = Router(data)
  result, route = router.doRoute(node1, node2)
  if result == 'success':
    # list the nodes
    print(route)

    # list the lat/long
    for i in route:
      node = data.rnodes[i]
      print("%d: %f,%f" % (i,node[0],node[1]))
  else:
    print("Failed (%s)" % result)

