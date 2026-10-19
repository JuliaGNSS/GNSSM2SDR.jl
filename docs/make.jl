using Documenter, GNSSM2SDR, M2SDRLoop

makedocs(;
    sitename = "GNSSM2SDR.jl",
    format = Documenter.HTML(; prettyurls = false),
    modules = [GNSSM2SDR, M2SDRLoop],
    # Nothing here runs without a board: the examples are shown, not doctested.
    doctest = false,
    checkdocs = :exports,  # only complain about undocumented *exported* symbols
    pages = [
        "index.md",
        "setup.md",
        "receiver.md",
        "loop_process.md",
        "emulator.md",
        "api.md",
    ],
)

deploydocs(; repo = "github.com/JuliaGNSS/GNSSM2SDR.jl.git", push_preview = true)
