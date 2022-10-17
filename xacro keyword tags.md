
# xacro:property / property
  - used to **create 'variables'** in the xacro file
  - these 'variables' are given a name and a value. They can be used throughout their scope.
  - syntax - `<xacro:property name="var_name" value="var_value"/>`
  
  - example - `<xacro:property name="wheel_radius" value="0.090"/>`
  
  - deployment - `<element_name attr_1="${2*wheel_radius}"/>`
  
  
# xacro:include / include
  - used to **include other xacro files** in the current xacro file.
  - similar to #include in C++ or import in python
  
  - syntax - `<xacro:include filename="path_of_included_file"/>`
  
  - example - `<xacro:include filename="$(find bot_desc_pkg)/urdf/bot.xacro"/>`
  
  - deployment - `<xacro:macro_name_from_included_file attr_1="some_value"/>`
  
  - as apparent, deploment can be done as if the the contents of the icluded file are 
  present in the current xacro file, however, to avoid confusions that might arise if there are macros or
  properties with the same name in both the files, create a namespace for the included files like so
  
  - syntax - `<xacro:include filename="path_of_included_file" ns="namespace"/>`
  - usage of namespace - `${namespace.property}`
  
  
  
# xacro:macro / macro
  - macros are **used to create element blocks** that can be used to ceate multiple instances of element blocks
  of such format/structure in simple calls.
  - They are very similar to classes in C++ or python.
  ```
  - syntax - <xacro:macro name="macro_name" params="param_1 param_2 *param_3"/>
                <element_1>
                <element_2>
                <element_3>
             </xacro:macro>
             
  - example - `<xacro:macro name="inertial_matrix" params="mass">
                  <inertial>
                    <mass value="${mass}" />
                    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="0.5" iyz="0.0" izz="1.0" />
                  </inertial>
              </xacro:macro>`    
  ```      
  - deployment syntax - `<xacro:macro_name param_1="val_1" param_2="val_2" param_3="val_3"/>`  
  - deployment example - `<xacro:inertial_matrix mass="1"/>`
  
  
  
# xacro:insert_block / insert_block
  - used to insert a block of elements in a single line in the xacro file. 
  - If a block of code can be defined as a property, then the insert_block tag can be used to place that block of elements in     multiple places within the xacro. Needles to say, any changes in the block of elements will be reflected throughout the listing while making the urdf.
  - syntax - `<xacro:insert_block name="block_name"/>`
  ```
  - deployment example 1 - <xacro:property name="pikachu_property">
                                <origin xyz="0 0 0" rpy="0 0 0"> 
                           </xacro:property>                           
                           <xacro:insert_block name="pikachu_property"/>```
                           
  - deployment example 2 - <xacro:macro name="pikachu_macro" params="param_1 *block_1" >
                                <element_1>
                                <element_2>
                                <xacro:insert_block name="block_1">
                            </xacro:macro>
```
xacro:arg / arg
xacro:element
xacro:name
xacro:attribute
xacro:if / if
xacro:unless / unless

xacro:eval-comments

xacro:call maybe? not sure