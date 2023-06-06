include("./optitrack.jl")
using StaticArrays, GLMakie
using FileIO: load
using GeometryBasics
using JSON
using RobotDynamics: RobotDynamics, translate_and_rotate_mesh_plot!

##########################
# Start optitrack
##########################
println("Start the python optitrack service, then press enter...")
readline()
OptiTrack.start()

##########################
# Create figure
##########################
Makie.inline!(false)
fig = Makie.Figure()
display(fig)
ax = Makie.Axis3(fig[1,1]; aspect = :data, limits = (-1.0, 1.0, -1.0, 1.0, 0.0, 1.0))

offset = SVector(0.0, 0.0, 0.0) # Temp value
o_lines = Observable(Vector{SVector{3, Float64}}())
o_point = Observable(offset)
points = [
    SVector(0.1, 0.0, 0.0),
    SVector(0.0, 0.0, 0.0),
    SVector(0.0, 0.1, 0.0),
    SVector(0.0, 0.0, 0.0),
    SVector(0.0, 0.0, 0.1)
]
lines!(ax, o_lines)
scatter!(ax, o_point)

function plotloop()
    println("Entering plot loop. Ctrl+C to continue.")
    try
        while true
            sleep(0.05)
            tf = OptiTrack.get_optitrack_transform()
        
            empty!(o_lines[])
            for i in eachindex(points)
                push!(o_lines[], tf * points[i])
            end
            notify(o_lines)
            o_point[] = tf * offset
        end    
    catch InterruptException
        nothing
    end
end

############################
# Calibrate tool and plot location
###########################
p, offset = OptiTrack.find_stationary_point()
scatter!(ax, p)
plotloop()

###########################
# Register location of bone
###########################
println("Registering femoral head points.")
pA, pB = OptiTrack.record_points(JSON.parsefile("/home/daniellarby/data/femoralhead.json"), offset)
tf, ret... = OptiTrack.compute_matching_transform(pA, pB)

###########################
# Plot bone in correct position
###########################
bone_plot = let
    bone_mesh = load("/home/daniellarby/data/bone_dec_stand.stl")
    coordinates(bone_mesh)
    1e-3*coordinates(bone_mesh)
    normal_bone_mesh = normal_mesh(GeometryBasics.Mesh(coordinates(bone_mesh), faces(bone_mesh)))
    plot_obj = mesh!(ax, bone_mesh, color=:red)
    # scale!(plot_obj, fill(1e-3, 3)...)
    Makie.scale(plot_obj)[] = (1e-3, 1e-3, 1e-3)
    plot_obj
end
translate_and_rotate_mesh_plot!(bone_plot, tf)
begin
    # Update limits
    o = RobotDynamics.origin(tf)
    a, b = o.-0.2, o.+0.2
    ax.limits[] = ((a[1], b[1]), (a[2], b[2]), (a[3], b[3]))
end
plotloop()

# OptiTrack.stop()
println("To end, do 'OptiTrack.stop(); exit()'")