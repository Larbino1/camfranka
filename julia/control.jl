using Revise
using RobotDynamics
using StaticArrays
using GLMakie

struct Pipes
    inpipe::IOStream
    outpipe::IOStream
end

function parse_robot_state(str::AbstractString)
    float_strs = split(str, ',')
    floats = map(float_strs) do s
        parse(Float64, s)
    end
    SVector{14, Float64}(floats)
end

function parse_robot_state(inpipe::IOStream; timeout)
    start = time()
    while true
        line = readline(inpipe)
        if (line != "")
            return parse_robot_state(line)
        end
        if (time() - start) > timeout
            throw(ErrorException("Timeout >$(timeout) seconds reading torques. Last read line was $(line)"))
        end
    end
end
parse_robot_state(p::Pipes, args...;kwargs...) = parse_robot_state(p.inpipe, args...; kwargs...)

function write_robot_torques(outpipe::IOStream, torques::SVector{7, Float64})
    write(outpipe, join(torques, ","))
    write(outpipe, "\n")
    flush(outpipe)
end
write_robot_torques(p::Pipes, args...) = write_robot_torques(p.outpipe, args...)

function open_communication(q_idxs)
    pipes = Pipes(open("cxxjuliapipe", "r"), open("juliacxxpipe", "w"))
    write_robot_torques(pipes, SVector{7}(0.,0.,0.,0.,0.,0.,0.))
    q_init = parse_robot_state(pipes; timeout=10.0)[q_idxs]
    pipes, q_init
end

function warmup(loop_func, args...)
    loop_func(0, args...)
    println("Loop times:     (should be less than 1ms)")
    @time(loop_func(0, args...))
    @time(loop_func(1, args...))
end

function mainloop(loop_func, args...)
    i = 0
    while true
        i += 1
        args = loop_func(i, args...)
    end
end

#######################
# Plotting
##

function plot(mechanism)
    # Plotting
    fig = Makie.Figure()
    ax = Axis3(fig[1, 1]; aspect = :data, limits = (-1.0, 1.0, -1.0, 1.0, 0.0, 1.0))
    display(fig)
    q_plot = Observable(@SVector(zeros(7)))
    m = compile(mechanism)
    robotplot!(ax, m, q_plot, frames(mechanism))
    robotplot!(ax, m, q_plot, visuals(mechanism))    
    fig, ax, q_plot
end

function plot_loop()
    println("Entering idle loop to plot robot. Ctrl+C to continue...")
    try
        while true
            sleep(.0001)
        end
    catch InterruptException
        println("\nContinuing...")
    end
end