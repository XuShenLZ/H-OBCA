#!/usr/bin/env julia

println("Starting H-OBCA Node")
# Pkg for saving and loading data from file
using JLD
# ##########################################

# Ask whether to solve the trajectories from scratch 
# or load it from files
is_solve = true

while true
    println("Solve the trajectories (y) or load from file (n)?: ")
    input = readline()

    if input == "y"
    	is_solve = true
    	break
	elseif input == "n"
		is_solve = false
		break
	else
		println("Invalid Input.")
	end

end

# ###########################################
# If we need to solve the trajectories
if is_solve
	# Run the julia code as described in the doc
	println("Setup Started...")

	include("setup.jl")

	println("Setup finished.")
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
	# Parameter selection and solving

	println("solving trajectories...")

	# Initialize dictionaries
	path = Dict()
	input = Dict()
	dt = Dict()

	# Make the choice HERE

	# Lower lane or upper lane
	for Start_lane in ["L", "U"]
		# Lower spot or upper spot
		for End_spot in ["L", "U"]
			# Forward parking or reverse
			for End_pose in ["F", "R"]
				println("New iteration...")
				println("Start_lane: ", Start_lane)
				println("End_spot: ", End_spot)
				println("End_pose: ", End_pose)

				# Obstacles for plotting
				lObPlot = spot_plot[End_spot]

				# Obstacles for optimization
				lOb = spot_opt[End_spot]

				# Obstacles for Hybrid A*
				ox, oy = obs_hybAstar(End_spot, l_map, w_lane, w_spot, l_spot)

				# Starting position
				x0 = [x0_x x0_y[Start_lane] x0_psi x0_v]

				# End position
				xF = [xF_x xF_y[[End_spot,End_pose]] xF_psi[[End_spot,End_pose]] xF_v]

				# ##########################################
				# Call the main.jl for solving

				xp10, up10, scaleTime10 = main(lObPlot, lOb, ox, oy, x0, xF)

				# Assign values
				path[[Start_lane, End_spot, End_pose]] = xp10
				input[[Start_lane, End_spot, End_pose]] = up10
				dt[[Start_lane, End_spot, End_pose]] = scaleTime10
			    
			end
		    
		end
	    
	end

	println("All maneuvers are solved!")
	# Save the variables to file, using JLD Pkg
	save("park_data.jld", "path",path, "input",input, "dt",dt)

	println("Data saved to file")

# If we just load data from file
else

	println("Loading variables from file...")

	# Load data from file, using JLD Pkg
	path = load("park_data.jld", "path")
	input = load("park_data.jld", "input")
	dt = load("park_data.jld", "dt")

	println("Successfully loaded")

end


# ##########################################
# Ros interface
using RobotOS
@rosimport parking.srv: maneuver
@rosimport parking.msg: car_state, car_input
rostypegen()
using parking.srv
using parking.msg

# ##########################################
# ROS server

function handle_request(req)
    println("Request:", req)
    # The same order as 'parking->maneuver.srv'
    request = [req.Start_lane, req.End_spot, req.End_pose]
    # Check whether the request is valid
    succeed = haskey(path, request)

	state_val = car_state()
	input_val = car_input()
    if succeed
    	state_val.x   = path[request][1, :]
    	state_val.y   = path[request][2, :]
    	state_val.psi = path[request][3, :]
    	state_val.v   = path[request][4, :]
    	input_val.delta = input[request][1, :]
    	input_val.acc   = input[request][2, :]
		respond = [succeed, state_val, input_val, dt[request]]
    else
    	println("The request is invalid")
    	respond = [succeed, state_val, input_val, [0]]
    end
    println("returning value...")
	return respond
end

function ros_server()
	# Initialize the server
    init_node("HOBCA_maneuver_server")
    s = Service{maneuver}("park_maneuver", handle_request)
    println("H-OBCA is ready to send maneuver")
    spin()

end

ros_server()
