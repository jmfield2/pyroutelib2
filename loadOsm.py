#!/etc/env python
#----------------------------------------------------------------
# load OSM data file into memory
#
#------------------------------------------------------
# Copyright 2007, Oliver White
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
# Changelog:
#  2007-11-04  OJW  Modified from pyroute.py
#  2007-11-05  OJW  Multiple forms of transport
#------------------------------------------------------
import os
import re
import sys
import math
import osmapi

# from pyroutelib2 import (tiledata, tilenames, weights)
import tiledata
import tilenames
import weights

class LoadOsm:
  """Parse an OSM file looking for routing information, and do routing with it"""
  def __init__(self, transport):
    """Initialise an OSM-file parser"""
    self.routing = {}
    self.rnodes = {}
    self.ways = {}
    self.transport = transport
    self.tiles = {}
    self.weights = weights.RoutingWeights()
    self.api = osmapi.OsmApi(api="api.openstreetmap.org")

  def distance(self,n1,n2):
    """Calculate distance between two nodes"""
    lat1 = float(n1[0]) * math.pi/180
    lon1 = float(n1[1]) * math.pi/180
    lat2 = float(n2[0]) * math.pi/180
    lon2 = float(n2[1]) * math.pi/180
    # TODO: projection issues
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    dist2 = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * (math.sin(dlon/2))**2
    dist2 = 2 * math.asin( math.sqrt(dist2) )
    dist = 6371000 * dist2 # meters
    return(dist)
 
  def getArea(self, lat, lon):
    """Download data in the vicinity of a lat/long.
    Return filename to existing or newly downloaded .osm file."""
    
    z = tiledata.DownloadLevel()
    (x,y) = tilenames.tileXY(float(lat), float(lon), z)

    tileID = '%d,%d'%(x,y)
    if(self.tiles.get(tileID,False)):
      #print "Already got %s" % tileID
      return
    self.tiles[tileID] = True
    
    filename = tiledata.GetOsmTileData(z,x,y)
    #print "Loading %d,%d at z%d from %s" % (x,y,z,filename)
    return(self.loadOsm(filename))

  def loadOsm(self, filename):
    if(not os.path.exists(filename)):
      print("No such data file %s" % filename)
      return(False)

    nodes, self.ways = {}, {}

    with open(filename, "r") as fp:
      osm_xml = fp.read()
    if len(osm_xml.strip()) < 1:
      print("No data read from {}".format(filename))
      return(False)

    #data = self.api.ParseOsm(osm_xml) # minidom sucks
    from lxml import etree
    tree = etree.parse(open(filename, "r+"))
    root = tree.getroot()
    result = []

    for line in root:
    	if line.tag == "node":
                result.append({u"type": line.tag, u"data": line.attrib})		
	elif line.tag == "way":
		d = dict(line.attrib)
		d['nd'] = [x.attrib['ref'] for x in line.findall('nd')]
		d['tag'] = {x.attrib['k']: x.attrib['v'] for x in line.findall('tag')}
                result.append({u"type": line.tag, u"data": d})
	elif line.tag == "relation":
                result.append({u"type": line.tag, u"data": line.attrib})

    data = result
    # data = [{ type: node|way|relation, data: {}},...]

    for x in data:
      try:
        if x['type'] == 'node':
          nodes[x['data']['id']] = x['data']
        elif x['type'] == 'way':
          self.ways[x['data']['id']] = x['data']
        else:
          continue
      except KeyError:
        # Don't care about bad data (no type/data key)
        continue
    #end for x in data
    for way_id, way_data in self.ways.items():
      way_nodes = []
      for nd in way_data['nd']:
        if nd not in nodes:
          continue
        way_nodes.append([nodes[nd]['id'], nodes[nd]['lat'], nodes[nd]['lon']])
      self.storeWay(way_id, way_data['tag'], way_nodes)

    return(True)
  
  def storeWay(self, wayID, tags, nodes):
    highway = self.equivalent(tags.get('highway', ''))
    railway = self.equivalent(tags.get('railway', ''))
    oneway = tags.get('oneway', '')
    reversible = not oneway in('yes','true','1')
    parking = tags.get('amenity', 'None')
    building = tags.get('building', 'no')

    if len(parking) > 0 and parking in ('parking'):
      if self.transport == 'foot': highway = 'footway'
      elif self.transport == 'car': highway = 'unclassified'
    if len(building) > 0 and building in ('yes'):
      if self.transport == 'foot': highway = 'footway'

    # Calculate what vehicles can use this route
    # TODO: just use getWeight != 0
    access = {}
    access['cycle'] = highway in ('primary','secondary','tertiary','unclassified','minor','cycleway','residential', 'track','service')
    access['car'] = highway in ('motorway','trunk','primary','secondary','tertiary','unclassified','minor','residential', 'service')
    access['train'] = railway in('rail','light_rail','subway')
    access['horse'] = highway in ('track','unclassified','bridleway')
    access['foot'] = access['cycle'] or highway in('footway','steps','landing') 

    # Store routing information
    last = [None,None,None]

    if(wayID == 41 and 0):
      print(nodes)
      sys.exit()
    for node in nodes:
      (node_id,x,y) = node
      if last[0]:
        if(access[self.transport]):
          weight = self.weights.get(self.transport, highway)
          self.addLink(last[0], node_id, weight)
          self.makeNodeRouteable(last)
          if reversible or self.transport == 'foot':
            self.addLink(node_id, last[0], weight)
            self.makeNodeRouteable(node)
      last = node

  def makeNodeRouteable(self,node):
    self.rnodes[node[0]] = [node[1],node[2]]
    
  def addLink(self,fr,to, weight=1):
    """Add a routeable edge to the scenario"""
    try:
      if to in list(self.routing[fr].keys()):
        return
      self.routing[fr][to] = weight
    except KeyError:
      self.routing[fr] = {to: weight}

  def equivalent(self,tag):
    """Simplifies a bunch of tags to nearly-equivalent ones"""
    equivalent = { \
      "primary_link":"primary",
      "trunk":"primary",
      "trunk_link":"primary",
      "secondary_link":"secondary",
      "tertiary":"secondary",
      "tertiary_link":"secondary",
      "residential":"unclassified",
      "minor":"unclassified",
      "steps":"footway",
      "driveway":"service",
      "pedestrian":"footway",
      "bridleway":"cycleway",
      "track":"cycleway",
      "arcade":"footway",
      "canal":"river",
      "riverbank":"river",
      "lake":"river",
      "light_rail":"railway"
      }
    try:
      return(equivalent[tag])
    except KeyError:
      return(tag)

  def is_disconnected(self, node):
  	""" Checks 'connectedness' of node to the rest of the graph...does a simple BFS end within X iterations? """
  	if node not in self.routing: return False
  	q = [node]
  	v = set(node)
  	cnt = 0
  	while len(q) > 0 and cnt < 50:
  		cnt += 1
  		nd = q.pop(0)
  		for x in self.routing[nd].keys():
  			if x not in self.routing: continue
  			if x in v: continue
  			q.append(x)
  			v.add(x)

  	if len(v) <= 20: return True
  	return False

  def findNode(self,lat,lon):
    """Find the nearest node that can be the start of a route"""
    self.getArea(lat,lon)
    maxDist = 1E+20
    nodeFound = None
    posFound = None
    for (node_id,pos) in list(self.rnodes.items()):
      dy = float(pos[0]) - lat
      dx = float(pos[1]) - lon
      dist = dx * dx + dy * dy

      if(dist < maxDist and not self.is_disconnected(node_id)):
        maxDist = dist
        nodeFound = node_id
        posFound = pos
    # print("found at %s"%str(posFound))
    return(nodeFound)

  def findNodes(self,lat,lon):
    """Find the nearest node that can be the start of a route"""
    self.getArea(lat,lon)
    maxDist = 20 # meters 
    nodes = {}
    for (node_id,pos) in list(self.rnodes.items()):
      dy = float(pos[0]) - lat
      dx = float(pos[1]) - lon
      dist = self.distance(pos, [lat, lon])
      if(dist < maxDist):
	nodes[node_id] = dist #"%s,%s" % (self.rnodes[node_id][0], self.rnodes[node_id][1])] = dist
    # print("found at %s"%str(posFound))
    import operator
    n_x = sorted(nodes.items(), key=operator.itemgetter(1))
    return(dict(n_x))

      
  def report(self):
    """Display some info about the loaded data"""
    print("Loaded %d nodes" % len(list(self.rnodes.keys())))
    print("Loaded %d %s routes" % (len(list(self.routing.keys())), self.transport))

# Parse the supplied OSM file
if __name__ == "__main__":
  data = LoadOsm("cycle")
  if(not data.getArea(29.738632, -95.404546)):
    print("Failed to get data")
  data.getArea(29.738632, -95.404546)
  data.report()

  print("Searching for node: found " + str(data.findNode(52.55291,-1.81824)))
