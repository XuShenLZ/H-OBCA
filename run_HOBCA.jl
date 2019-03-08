# ##########################################
# Run the julia code as described in the doc
# include("setup.jl")
# ##########################################

# ##########################################
# Map Parameters

# The length of the map
l_map = 10

# Lane width
w_lane = 3

# Spot width
w_spot = 1.3
l_spot = 5

# Obstacles
# Vertices are given CLOCK-WISE
spot_plot = Dict(
	# The lower parking position
	"L"=>
	[ 
	[ [-l_map;-w_lane], [-w_spot;-w_lane], [-w_spot;-w_lane-l_spot], [-l_map;-w_lane-l_spot], [-l_map;-w_lane] ],
	[ [w_spot;-w_lane], [l_map;  -w_lane], [l_map;  -w_lane-l_spot], [w_spot;-w_lane-l_spot], [w_spot;-w_lane] ],
	[ [-l_map;w_lane+l_spot], [l_map;w_lane+l_spot], [l_map;w_lane], [-l_map;w_lane], [-l_map;w_lane+l_spot] ] ]
	,
	# The upper parking position
	"U"=>
	[
	[ [-l_map; w_lane+l_spot], [-w_spot; w_lane+l_spot], [-w_spot; w_lane], [-l_map; w_lane], [-l_map; w_lane+l_spot] ],
	[ [ w_spot; w_lane+l_spot], [ l_map; w_lane+l_spot], [l_map; w_lane], [ w_spot; w_lane], [ w_spot; w_lane+l_spot] ],
	[ [-l_map;-w_lane], [l_map;-w_lane], [l_map;-w_lane-l_spot], [-l_map;-w_lane-l_spot], [-l_map;-w_lane] ] ]
)

# Vertices are given CLOCK-WISE
spot_opt = Dict(
	# The lower parking position
	"L"=>
	[ 
	[ [-l_map;-w_lane], [-w_spot;-w_lane], [-w_spot;-w_lane-l_spot] ],
	[ [w_spot;-w_lane-l_spot], [w_spot;-w_lane], [l_map;  -w_lane]  ],
	[ [l_map;w_lane], [-l_map;w_lane] ] ]
	,
	"U"=>
	[ 
	[ [-w_spot; w_lane+l_spot], [-w_spot; w_lane],[-l_map; w_lane]  ], 
	[ [l_map;   w_lane], [w_spot; w_lane], [w_spot; w_lane+l_spot]  ],
	[ [-l_map;-w_lane], [l_map;-w_lane] ] ]
)

# The function to build obstacle representation for Hybrid A*
function obs_hybAstar(spot_pos, l_map, w_lane, w_spot, l_spot)

	ox = Float64[]
	oy = Float64[]

	if spot_pos == "L"
		# obstacle 1
		for i = -l_map:0.1:-w_spot
		    push!(ox, Float64(i))
		    push!(oy, -w_lane)
		end
		for i in -2-w_lane-l_spot:-w_lane
		    push!(ox, -w_spot)
		    push!(oy, Float64(i))
		end
		# obstacle 2
		for i in -2-w_lane-l_spot:-w_lane
		    push!(ox, w_spot)
		    push!(oy, Float64(i))
		end
		for i = w_spot:0.1:l_map
		    push!(ox, Float64(i))
		    push!(oy, -w_lane)
		end
		# obstacle 3
		for i = -l_map:l_map
		    push!(ox, Float64(i))
		    push!(oy, w_lane)
		end

	elseif spot_pos == "U"
	    # obstacle 1
		for i = -l_map:0.1:-w_spot
		    push!(ox, Float64(i))
		    push!(oy, w_lane)
		end
		for i in w_lane:2+w_lane+l_spot
		    push!(ox, -w_spot)
		    push!(oy, Float64(i))
		end
		# obstacle 2
		for i in w_lane:2+w_lane+l_spot
		    push!(ox, w_spot)
		    push!(oy, Float64(i))
		end
		for i = w_spot:0.1:l_map
		    push!(ox, Float64(i))
		    push!(oy, w_lane)
		end
		# obstacle 3
		for i = -l_map:l_map
		    push!(ox, Float64(i))
		    push!(oy, -w_lane)
		end
	else
		println("The target spot is not correctly specified")
	end

	return ox, oy
    
end

# Start position
x0_x   = -6
# Y position is a dictionary
# Upper and lower Spot
x0_y   = Dict("U"=>w_lane/2, "L"=>-w_lane/2)
x0_psi = 0
x0_v   = 0

# End position
xF_x   = 0
# Y position is a dictionary
# Upper and lower Spot
# Forward and reverse parking
xF_y   = Dict(["U","F"]=> w_lane+1,
			  ["U","R"]=> w_lane+l_spot-1,
			  ["L","F"]=>-w_lane-1,
			  ["L","R"]=>-w_lane-l_spot+1)

xF_psi = Dict(["U","F"]=> pi/2,
			  ["U","R"]=>-pi/2,
			  ["L","F"]=>-pi/2,
			  ["L","R"]=> pi/2)
xF_v   = 0

# ##########################################
# Parameter selection

# Make the choice HERE
Start_lane = "L"
End_spot = "L"
End_pose = "F"

# Obstacles for plotting
lObPlot = spot_plot[End_spot]

# Obstacles for optimization
lOb = spot_opt[End_spot]

# Obstacles for Hybrid A*
ox, oy = obs_hybAstar(End_spot, l_map, w_lane, w_spot, l_spot)

# Lane width
w_lane = 3

# Spot width
w_spot = 1.3
l_spot = 10

# Starting position
x0 = [x0_x x0_y[Start_lane] x0_psi x0_v]

# End position
xF = [xF_x xF_y[[End_spot,End_pose]] xF_psi[[End_spot,End_pose]] xF_v]

# ##########################################
# Call the main.jl for solving
include("main.jl")
