include("./control.jl")


function build_zigzag_spline(; w=0.0025, x0=0.0, L=0.05, h=0.0, Nk=5, Nr=25)
    forward = Vector(LinRange(x0, x0 + L, Nk))
    backward = Vector(reverse(forward))
    spline = Matrix{Float64}(undef, 0, 3)

    for i = 0:(Nr-1)
        knots = (i % 2) != 1 ? forward : backward
        spline = vcat(
            spline,
            hcat(knots, w * i * ones(size(knots)), h * ones(size(knots)))
        )
    end
    RobotDynamics.CubicSpline(spline)
end

function build_square_spline(; w=0.075, N=5)
    spline = [
        0.0 0.0 0.0;
        0.0 0.5 0.0;
        0.0 1.0 0.0;
        0.5 1.0 0.0;
        1.0 1.0 0.0;
        1.0 0.5 0.0;
        1.0 0.0 0.0;
        0.5 0.0 0.0;
    ]
    spline .*= w # 5cm length
    RobotDynamics.LoopingCubicSpline(spline)
end

function build_cardiod(; w=0.1)
    spline = [
         0.0 0.0 0.0;
         0.2 0.2 0.0;
         0.5 0.5 0.0;
         0.5 1.0 0.0;
         0.1 0.8 0.0;
        -0.1 0.8 0.0;
        -0.5 1.0 0.0;
        -0.5 0.5 0.0
        -0.2 0.2 0.0;
    ]
    spline .*= w # 5cm length
    RobotDynamics.LoopingCubicSpline(spline)
end

function build_spiral_spline(; r1=0.04, r2=0.08, N=100, Nrot=10, h=0.1)
    spline = Matrix{Float64}(undef, (N, 3))

    for i = 1:N
        t = 2π*(i-1)/N
        ω1 = 2 * t
        ω2 = 2*Nrot * t
        r = r1 + (r2 - r1)*(1.0 + sin(ω1))/2
        spline[i, 1] = r * sin(ω2)
        spline[i, 2] = r * (cos(ω2) - 1.0) # To make start 0.0
        spline[i, 3] = h*(1.0 - cos(ω1))
    end
    RobotDynamics.LoopingCubicSpline(spline)
end



function add_cart!(mechanism, initial_ee_transform, spline)
    # Spline joint
    cart_frame = add_frame!(mechanism, "cart_frame")
    TR = Transform(origin(initial_ee_transform), zero(Rotor))
    RailIdx = ndof(mechanism) + 1

    rail_joint = Rail(spline, TR, RailIdx)
    add_joint!(mechanism, rail_joint, root_frame(mechanism), cart_frame)
    # Joint Inertia/damping
    push!(mechanism, JointInerter(0.2, RailIdx))
    push!(mechanism, JointDamper(20.0, RailIdx))

    # EE Spring/Damper
    stiffness = @SMatrix([2000.0 0.0    0.0
                          0.0    2000.0 0.0
                          0.0    0.0    2000.0])
    damping = @SMatrix([5.0 0.0 0.0
                        0.0 5.0 0.0
                        0.0 0.0 5.0])
    EE_frame = find_frame_index(mechanism, "burr_frame")
    push!(mechanism, LinearSpring(stiffness, cart_frame, EE_frame))
    push!(mechanism, LinearDamper(damping, root_frame(mechanism), cart_frame))
    mechanism
end

function shape_demo(spline)
    # Load mechanism and remove inertial components so that they do not contribute
    # to computed generalized_force when we have constructed the control mechanism
    mechanism = parseRSON("../../RSONs/rsons/franka_panda/pandaDremelMount.rson")
    empty!(inertances(mechanism))
    empty!(generic_components(mechanism))

    EE_frame = find_frame_index(mechanism, "burr_frame")

    # Get starting position for spline
    # q_robot_init = @SVector(zeros(7))
    pipes, q_robot_init = open_communication(@SVector([i for i in 1:7]))
    initial_EE_transform = kinematics(compile(mechanism), q_robot_init)[EE_frame]
    add_cart!(mechanism, initial_EE_transform, spline)
    
    # fig = Makie.Figure()
    # ax = Makie.Axis3(fig[1,1]; aspect=:data)
    # RobotDynamics.Splines.splineplot!(ax, joints(mechanism)[end].spline)
    # display(fig)

    # Get useful things
    cart_frame = find_frame_index(mechanism, "cart_frame")

    # Compile mechanism
    m = compile(mechanism)

    # Initial state
    q_robot, q̇_robot = q_robot_init, @SVector(zeros(7))
    q_virtual, q̇_virtual = SVector(0.0), SVector(0.0)
    u_virtual = SVector(0.0)

    # Safety check...
    begin
        q_init = vcat(q_robot, q_virtual)
        q̇_init = vcat(q̇_robot, q̇_virtual)
        initial_transforms = kinematics(m, q_init)
        # @show initial_transforms
        ee_init = origin(initial_transforms[EE_frame])
        cart_init = origin(initial_transforms[cart_frame])
        @show ee_init, cart_init
        @assert all(ee_init - cart_init .≈ 0.0)

        # hessianresult = RobotDynamics.compute_hessianresult(m.rbtree, q_init)
        # @show RobotDynamics.jacobian(hessianresult)[cart_frame]
        # @show RobotDynamics.hessian(hessianresult)[cart_frame]

        # @show q_init, q̇_init
        # @show generalized_force(m, q_init, q̇_init)
        @assert all(generalized_force(m, q_init, q̇_init) .≈ 0.0)
    end

    uʳ_idxs = @SVector([i for i in 1:7])
    uᶜ_idxs = @SVector([i for i in 8:8])

    function loop_func(i, q_robot, q̇_robot, q_virtual, q̇_virtual)
        # Create full state vector
        q = vcat(q_robot, q_virtual)
        q̇ = vcat(q̇_robot, q̇_virtual)

        # Compute forces
        u = generalized_force(m, q, q̇)
        u_robot, u_virtual = u[uʳ_idxs], u[uᶜ_idxs]

        # SIMULATE virtual mechanism
        M = inertance_matrix(m, q, q̇)
        M_virtual = M[uᶜ_idxs, uᶜ_idxs]
        q̈_virtual = M_virtual \ u_virtual
        q_virtual += 1e-3 * q̇_virtual
        q̇_virtual += 1e-3 * q̈_virtual

        # Send forces to actual robot
        write_robot_torques(pipes, u_robot)

        if ((i % 100) == 0)
            println(q_virtual)
        end

        # Read robot state
        q_robot, q̇_robot = parse_robot_pos_vel(pipes; timeout=0.2)
        q_robot, q̇_robot, q_virtual, q̇_virtual
    end

    args = (q_robot, q̇_robot, q_virtual, q̇_virtual)
    warmup(loop_func, args...)
    println("Ready...")
    try
        mainloop(loop_func, args...)
    catch e
        println(e)
    end
end

function debugging(spline)
    fig = Makie.Figure()
    ax = Makie.Axis3(fig[1, 1]; aspect=:data)
    # spline = build_spiral_spline()
    RobotDynamics.Splines.splineplot!(ax, spline)
    display(fig)
    # @show RobotDynamics.Splines.position(0.57, spline)
    # RobotDynamics.Splines.position(8.57, spline)
    # @show RobotDynamics.Splines.position(8.57, spline)
end

# spline = build_square_spline(w=0.2)
spline = build_cardiod(w=0.2)
# spline = build_spiral_spline()  

# @show RobotDynamics.Splines.position(0.0, spline)
shape_demo(spline)
debugging(spline)
