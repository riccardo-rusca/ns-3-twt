set(name report-manager)

set(source_files
    model/ReportManager.cc
)

set(header_files
    model/ReportManager.h
)

set(libraries_to_link
    ${libcore} ${libconfig-store} ${libstats}
)

set(test_sources
)

build_lib("${name}" "${source_files}" "${header_files}" "${libraries_to_link}"
        "${test_sources}"
)
