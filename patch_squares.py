from pathlib import Path

path = Path('C:/Users/15188/Downloads/IE 482-582/spring2026/Projects/gazebo_demo/worlds/new_warehouse_world.sdf')
text = path.read_text()

# Define the white square visual
white_square = '''        <visual name="white_square">
          <geometry><box><size>0.02 0.1 0.1</size></box></geometry>
          <material>
            <ambient>1.0 1.0 1.0 1</ambient>
            <diffuse>1.0 1.0 1.0 1</diffuse>
          </material>
        </visual>\n'''

# For bottom markers (row 1), replace digit visuals with one white square
bottom_digit_block = '''        <visual name="digit_one_stem">
          <pose>0.032 0.000 0.000 0 0 0</pose>
          <geometry><box><size>0.02 0.070 0.340</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="digit_one_base">
          <pose>0.032 0.000 -0.180 0 0 0</pose>
          <geometry><box><size>0.02 0.250 0.045</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="digit_one_top">
          <pose>0.032 0.000 0.170 0 0 0</pose>
          <geometry><box><size>0.02 0.250 0.045</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>\n'''

bottom_right_digit_block = '''        <visual name="digit_one_stem">
          <pose>-0.032 0.000 0.000 0 0 0</pose>
          <geometry><box><size>0.02 0.070 0.340</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="digit_one_base">
          <pose>-0.032 0.000 -0.180 0 0 0</pose>
          <geometry><box><size>0.02 0.250 0.045</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="digit_one_top">
          <pose>-0.032 0.000 0.170 0 0 0</pose>
          <geometry><box><size>0.02 0.250 0.045</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>\n'''

# For top markers (row 2), replace with two white squares
top_digit_block = '''        <visual name="digit_two_top">
          <pose>0.032 0.000 0.170 0 0 0</pose>
          <geometry><box><size>0.02 0.400 0.045</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="digit_two_upper_right">
          <pose>0.032 0.160 0.075 0 0 0</pose>
          <geometry><box><size>0.02 0.045 0.220</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="digit_two_middle">
          <pose>0.032 0.000 0.000 0 0 0</pose>
          <geometry><box><size>0.02 0.400 0.045</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="digit_two_lower_left">
          <pose>0.032 -0.160 -0.095 0 0 0</pose>
          <geometry><box><size>0.02 0.045 0.220</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="digit_two_bottom">
          <pose>0.032 0.000 -0.190 0 0 0</pose>
          <geometry><box><size>0.02 0.400 0.045</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>\n'''

top_right_digit_block = '''        <visual name="digit_two_top">
          <pose>-0.032 0.000 0.170 0 0 0</pose>
          <geometry><box><size>0.02 0.400 0.045</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="digit_two_upper_right">
          <pose>-0.032 0.160 0.075 0 0 0</pose>
          <geometry><box><size>0.02 0.045 0.220</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="digit_two_middle">
          <pose>-0.032 0.000 0.000 0 0 0</pose>
          <geometry><box><size>0.02 0.400 0.045</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="digit_two_lower_left">
          <pose>-0.032 -0.160 -0.095 0 0 0</pose>
          <geometry><box><size>0.02 0.045 0.220</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
        <visual name="digit_two_bottom">
          <pose>-0.032 0.000 -0.190 0 0 0</pose>
          <geometry><box><size>0.02 0.400 0.045</size></box></geometry>
          <material>
            <ambient>0.02 0.02 0.02 1</ambient>
            <diffuse>0.02 0.02 0.02 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>\n'''

# Replace for left rack bottom markers
text = text.replace(bottom_digit_block, white_square)
# Replace for right rack bottom markers
text = text.replace(bottom_right_digit_block, white_square)
# Replace for left rack top markers
text = text.replace(top_digit_block, white_square + white_square.replace('white_square', 'white_square_2').replace('<pose>', '<pose>0.032 -0.05 0.000 0 0 0</pose>') + white_square.replace('white_square', 'white_square_3').replace('<pose>', '<pose>0.032 0.05 0.000 0 0 0</pose>'))
# Wait, better to define two squares for top.

# Actually, for top, two squares side by side.
two_squares = '''        <visual name="white_square_left">
          <pose>0.032 -0.05 0.000 0 0 0</pose>
          <geometry><box><size>0.02 0.1 0.1</size></box></geometry>
          <material>
            <ambient>1.0 1.0 1.0 1</ambient>
            <diffuse>1.0 1.0 1.0 1</diffuse>
          </material>
        </visual>
        <visual name="white_square_right">
          <pose>0.032 0.05 0.000 0 0 0</pose>
          <geometry><box><size>0.02 0.1 0.1</size></box></geometry>
          <material>
            <ambient>1.0 1.0 1.0 1</ambient>
            <diffuse>1.0 1.0 1.0 1</diffuse>
          </material>
        </visual>\n'''

two_squares_right = '''        <visual name="white_square_left">
          <pose>-0.032 -0.05 0.000 0 0 0</pose>
          <geometry><box><size>0.02 0.1 0.1</size></box></geometry>
          <material>
            <ambient>1.0 1.0 1.0 1</ambient>
            <diffuse>1.0 1.0 1.0 1</diffuse>
          </material>
        </visual>
        <visual name="white_square_right">
          <pose>-0.032 0.05 0.000 0 0 0</pose>
          <geometry><box><size>0.02 0.1 0.1</size></box></geometry>
          <material>
            <ambient>1.0 1.0 1.0 1</ambient>
            <diffuse>1.0 1.0 1.0 1</diffuse>
          </material>
        </visual>\n'''

text = text.replace(bottom_digit_block, white_square)
text = text.replace(bottom_right_digit_block, white_square)
text = text.replace(top_digit_block, two_squares)
text = text.replace(top_right_digit_block, two_squares_right)

path.write_text(text)
print('updated markers to white squares')
