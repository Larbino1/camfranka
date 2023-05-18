using StaticArrays

inpipe = open("cxxjuliapipe", "r")
outpipe = open("juliacxxpipe", "w")
println("Opened pipes")

function parse_robot_state(str::AbstractString)
    float_strs = split(str, ',')
    floats = map(float_strs) do s
        parse(Float64, s)
    end
    SVector{14, Float64}(floats)
end

function parse_robot_state()
    line = readline(inpipe)
    parse_robot_state(line)
end

function write_robot_torques(torques::SVector{7, Float64})
    write(outpipe, join(torques, ","))
    write(outpipe, "\n")
    flush(outpipe)
end

function test()
    write_robot_torques(SVector{7}(0.,1.,0.,1.,0.,1.,0.))
    println(parse_robot_state())

    while true
        println(parse_robot_state())
    end
end

test()

