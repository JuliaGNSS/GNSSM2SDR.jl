# The loop process's entry file, compiled with `juliac --trim=safe` (see
# build.sh) or run as a script: `julia --project=.. bin/gnss_loop.jl --csr=...`.
using M2SDRLoop

function (@main)(args::Vector{String})::Cint
    return M2SDRLoop.loop_main(args)
end
