from pathlib import Path

path = Path('C:/Users/15188/Downloads/IE 482-582/spring2026/Projects/gazebo_demo/worlds/new_warehouse_world.sdf')
text = path.read_text()

import re

insert = '''        <visual name="digit_one_top">
          <pose>0.032 0.000 0.170 0 0 0</pose>
          <geometry><box><size>0.02 0.250 0.045</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>\n'''
old_left = '''        <visual name="digit_one_base">
          <pose>0.032 0.000 -0.180 0 0 0</pose>
          <geometry><box><size>0.02 0.250 0.045</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>\n'''
old_right = '''        <visual name="digit_one_base">
          <pose>-0.032 0.000 -0.180 0 0 0</pose>
          <geometry><box><size>0.02 0.250 0.045</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>\n'''
new_left = old_left + insert + '      </link>'
new_right = old_right + insert + '      </link>'

# Remove any existing digit_one_top blocks before inserting fresh ones.
text = re.sub(r'\s*<visual name="digit_one_top">.*?</visual>\n', '', text, flags=re.S)
text = text.replace(old_left, new_left)
text = text.replace(old_right, new_right)
text = text.replace('<size>0.02 0.340 0.045</size>', '<size>0.02 0.400 0.045</size>')
text = text.replace('<size>0.02 0.045 0.190</size>', '<size>0.02 0.045 0.220</size>')
path.write_text(text)
print('patched new_warehouse_world.sdf')
