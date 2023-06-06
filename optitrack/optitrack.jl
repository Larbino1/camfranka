module OptiTrack

using JSON
using LinearAlgebra: norm, svd, det
using RobotDynamics: Transforms, Transform, Rotor
using StaticArrays
using GLMakie

using Base.Threads: @spawn, Atomic

@assert Threads.nthreads() > 1 "Number of threads is not greater than 1. Set the 'JULIA_NUM_THREADS' environment variable to increase this then relaunch julia."
Threads.nthreads()

pipe_name = "/tmp/cam_optitrack_pipe"
read_optitrack = Base.Threads.Atomic{Bool}(true)
tf_lock = ReentrantLock()
transform = Vector{Float64}(undef, 7)

function get_optitrack_transform()
    lock(tf_lock) do 
        Transform{Float64}(
            transform[SVector(1,2,3)],
            Rotor(transform[SVector(4,5,6)], transform[7])
        )
    end
end

function read_once(pipe)
    b = Vector{UInt8}(undef, 7)
    readbytes!(pipe, b, 56)
    floats = reinterpret(Float64, b)
    floats, eof(pipe)
    lock(tf_lock) do
        transform .= floats
    end
end

function read_optitrack_pipe(pipe)
    read_optitrack[] = true

    b = Vector{UInt8}(undef, 7)
    while read_optitrack[]
        readbytes!(pipe, b, 56; all=true)
        floats = reinterpret(Float64, b)
        # @show eof(pipe), floats
        lock(tf_lock) do
            transform .= floats
        end
    end
    close(pipe)
end

function start()
    println("Opening pipe...")
    p = open(pipe_name, "r")
    @show p
    @spawn read_optitrack_pipe(p)
end

function stop()
    read_optitrack[] = false
    nothing
end


###################################
# Procedures
###################################

function find_stationary_point(;T=10.0)
    println("Place the point of the stylus on a surface where you can rotate it without moving, then press enter.")
    readline()
    transforms = Vector{Transform{Float64}}()
    println("For the next $(T) seconds, slowly rotate the stylus through a wide range of positions without moving the tip.")

    start = time()
    while (time() - start) < T
        push!(transforms, get_optitrack_transform())
        sleep(0.01)
    end

    p_world, offset, θ, A, b = Transforms.find_stationary_point(transforms)

    residuals = A*θ - b
    errors = map(i -> SVector{3, Float64}(residuals[(i-1)*3+1:i*3]), 1:length(transforms))
    err_norms = map(norm, errors)
    max_err = maximum(err_norms)
    println("Offset is $(offset)")
    println("Maximum error was $(1e3*max_err)mm")
    println("Avg error was $(1e3*sum(err_norms)/length(err_norms))mm")

    p_world, offset
end

function register_point(offset::SVector{3, Float64}; T=0.2)
    sum_points = get_optitrack_transform() * offset
    N = 1
    start = time()
    while (time() - start) < T
        sum_points += get_optitrack_transform() * offset
        N += 1
        sleep(min(1/120, T/100))
    end
    sum_points/N
end

function row_avg(mat)
    N = size(mat, 2)
    val = mat[:, 1]
    for i = 2:N
        val += mat[:, i]
    end
    val ./ N
end

function match_frame(data_json, offset)
    json = JSON.parsefile(data_json)
    registration_points, registered_points = record_points(json, offset)
    # Match data
    compute_matching_transform(registration_points, registered_points)
end

function record_points(json, offset)
    points = json["points"]
    N = length(points)
    registration_points = Matrix{Float64}(undef, (3, N))
    registered_points = Matrix{Float64}(undef, (3, N))
    for (i, p) in enumerate(points)
        println("Registering point $(p["name"]). Press enter when in position...")
        readline()
        registration_points[:, i] .= p["data"]
        registered_points[:, i] .= register_point(offset)
    end
    registration_points, registered_points
end

function compute_matching_transform(pointsA, pointsB)
    comA = SVector{3}(row_avg(pointsA))
    comB = SVector{3}(row_avg(pointsB))
    p_A = pointsA .- comA
    p_B = pointsB .- comB
    
    N = size(pointsA, 2)

    tf = let
        H = zeros(3,3)
        for i = 1:N
            @show p_A[1:3, i] * p_B[1:3, i]'
            H += p_A[1:3, i] * p_B[1:3, i]'
        end
        (;U, V) = svd(H; full=true)
        # Flip direction of least significant vector, to turn into a rotation if a reflection is found.  
        # Required when the points are all coplanar.
        V[1:3, 2] = V[1:3, 2] * (det(V) * det(U))
        R = SMatrix{3, 3}(V*U') # TODO should this be U*V'??
        r = Rotor(R) # Convert rotation matrix to rotor

        @assert R*comA ≈ r*comA # Double check...
        @assert (det(R) ≈ 1) # If it is a rotation matrix, this should be true

        Transform(comB - r*comA, r)
    end

    # Compute errors...
    errs = map(1:size(p_A, 2)) do i
        SVector{3}(pointsB[:,i]) - tf*SVector{3}(pointsA[:,i])
    end
    dists = map(norm, errs)
    avg_error = sum(dists)/length(dists);
    max_error = maximum(dists);    
    # @show avg_error, max_error
    println("Average error for each point: $(1e3*avg_error)mm. Maximum error (worst point): $(1e3*max_error)mm")
    return tf, dists, avg_error, max_error
end





# std::cout << "Success?:" << pointMatchResult.success << std::endl;
# std::cout << "Rotation:" << std::endl << pointMatchResult.rotation << std::endl;
# std::cout << "Translation:" << std::endl << pointMatchResult.translation << std::endl;
# std::cout << "Mean distance:" << pointMatchResult.avg_error * 1000 << "mm." << std::endl;
# std::cout << "Max distance:" << pointMatchResult.max_error * 1000 << "mm." << std::endl;
# Eigen::Matrix4d transform; // Your Transformation Matrix
# transform.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
# transform.block<3,3>(0,0) = pointMatchResult.rotation;
# transform.block<3,1>(0,3) = pointMatchResult.translation;    
# if (YesNoPrompt("Save transform to cache? [y/n]")) {
#   save_frame_transform(transform_cache, Eigen::Affine3d(transform));
# }
# mesh_transform.matrix() = transform;


end