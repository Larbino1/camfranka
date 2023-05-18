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

function test()
    # Prepare fig
    # fig = Makie.Figure()
    # ax = Axis3(fig[1, 1]; aspect = :data, limits = (-1.0, 1.0, -1.0, 1.0, 0.0, 1.0))
    # display(fig)
    
    # # Load and plot mechanism
    # mechanism = RobotDynamics.RSON.parseRSON("/home/daniellarby/RSONs/rsons/franka_panda/pandaDummyInstrument.rson")
    # m = compile(mechanism)
    # q_plot = Observable(@SVector(zeros(7)))
    q_idxs = @SVector([i for i in 1:7])
    # robotplot!(ax, m, q_plot, visuals(mechanism))

    # Open communication
    pipes = Pipes(open("cxxjuliapipe", "r"), open("juliacxxpipe", "w"))
    write_robot_torques(pipes, SVector{7}(0.,0.,0.,0.,0.,0.,0.))
    q_init = parse_robot_state(pipes; timeout=10.0)[q_idxs]
    # println("q_init: $(q_init)")
    # q_plot[] = q_init
    # println("Set q_init on plot...")
    # sleep(.0001)
    
    function plot_func(q)
        # println("Plotting...")
        # q_plot[] = q
        println(q)
        # sleep(.00001)
    end

    function loop_func(i, q_prev)
        # start = time()
        write_robot_torques(pipes, SVector{7}(0.,0.,0.,0.,0.,0.,0.))
        q = parse_robot_state(pipes; timeout=2e-3)[q_idxs]
        if ( (i % 100) == 0)
            plot_func(q)
        end
        q
    end


    loop_func(0, q_init)
    println("Loop time:     (should be less than 1ms)")
    @time(loop_func(1, q_init))
    println("Loop time with plot:     (should be less than 1ms)")
    @time(loop_func(0, q_init))
    
    println("Ready...")
    
    i = 0
    q = q_init
    while true
        i += 1
        q = loop_func(i, q)
    end
end

test()
