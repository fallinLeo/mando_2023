#!/usr/bin/env python3
#-*- coding: utf-8 -*-
import os
os.environ['SHAPE_RESTORE_SHX'] = 'YES'
import numpy as np
from pyproj import CRS
import pandas as pd
import geopandas as gpd

# offset = {'seoul':[962897.516413939,1958728.3104721],'kcity':[935504.1834692371,1915769.1316598575]}

def to_csv(file, s):
   gdf = gpd.read_file(file)

   coor_dict={}
   for i in range(len(gdf)):
      link = -gdf['fid'][i]-9
      data = gdf['geometry'][i].xy
      coor_dict[i] = {'x':data[0], 'y':data[1], 'link':link}

   wx=[]
   wy=[]
   wz=[]
   for i in range(len(coor_dict)):
      wx.append(coor_dict[i]['x'][0])
      wy.append(coor_dict[i]['y'][0])
      wz.append(coor_dict[i]['link'])
   return wx,wy,wz

file_path = '~/catkin_ws/src/waypoint_server/shp_files/'+'hitech_s.shp'

wx,wy,link = to_csv(file_path, 'seoul')

df = pd.DataFrame({'X-axis':wx,'Y-axis':wy,'Link':link})
df.to_csv('output.csv', index=False, mode='w', encoding='utf-8-sig')

#data=pd.read_csv('sample.csv')
#x= data[data['Link']==1]['X-axis']
