module CoulombForceTests
using Test, LinearAlgebra, StaticArrays
using ..SpaceAGORAMuJoCo
const M=SpaceAGORAMuJoCo

@testset "zero-slip Coulomb force supports subcritical load without creep" begin
    for mu in (.1,.4,.8), alpha in (-.99,-.5,0.,.5,.99), angle in (0.,.7,2.3)
        W=[1.4 .2 -.1; .2 2.1 .3; -.1 .3 1.8];Rn=.3
        expected=[10.,alpha*mu*10cos(angle),alpha*mu*10sin(angle)]
        H=W+Diagonal([Rn,0.,0.]);b=-H*expected
        f=M._coulomb_sticking_force(W,Rn,b)
        @test norm(f-expected)<1e-12
        @test norm(f[2:3])<=mu*f[1]
        @test norm((W*f+b)[2:3])<1e-12
        @test abs((W*f+b)[1]+Rn*f[1])<1e-12
        @test dot(f[2:3],zeros(2))==0.
    end
end

@testset "breakaway direction satisfies coupled Coulomb equations" begin
    for coupling in (0.,.2), mu in (.1,.4,.8), angle in range(0.,2pi;length=17)
        W=[1.4 coupling -.1; coupling 2.1 .3; -.1 .3 1.8];Rn=.3
        desired=[10.,1.5mu*10cos(angle),1.5mu*10sin(angle)]
        b=-(W+Diagonal([Rn,0.,0.]))*desired
        f=M._coulomb_breakaway_force(W,Rn,b,mu)
        a=(W*f+b)[2:3]
        @test f[1]>=0
        @test abs(norm(f[2:3])-mu*f[1])<1e-11
        @test norm(f[2:3]+mu*f[1]*a/norm(a))<1e-10
        @test abs((W*f+b)[1]+Rn*f[1])<1e-12
        @test dot(f[2:3],a)<0
    end
    W=Matrix{Float64}(I,3,3);Rn=.1
    for load in (-6.,6.)
        f=M._coulomb_breakaway_force(W,Rn,[-11.,load,0.],.4)
        @test f≈[10.,-sign(load)*4.,0.] atol=1e-12
    end
    for excess in (eps(Float64),1e-14,1e-10,1e-6), angle in (0.,pi/2,pi,3pi/2)
        b=[-11.,(4+excess)*cos(angle),(4+excess)*sin(angle)]
        f=M._coulomb_breakaway_force(W,Rn,b,.4)
        @test f[1]≈10. atol=1e-12
        @test norm(f[2:3])≈4. atol=1e-12
        @test norm(f[2:3]+4SVector(cos(angle),sin(angle)))<1e-12
        @test dot(f[2:3],(W*f+b)[2:3])<=1e-13
    end
end
end
