set(name applications)

set(source_files
    helper/bulk-send-helper.cc
    helper/on-off-helper.cc
    helper/packet-sink-helper.cc
    helper/three-gpp-http-helper.cc
    helper/udp-client-server-helper.cc
    helper/udp-echo-helper.cc
    helper/udp-measurement-client-server-helper.cc
    model/application-packet-probe.cc
    model/bulk-send-application.cc
    model/latency-comp-header.cc
    model/onoff-application.cc
    model/packet-loss-counter.cc
    model/packet-sink.cc
    model/seq-ts-echo-header.cc
    model/seq-ts-header.cc
    model/seq-ts-size-header.cc
    model/three-gpp-http-client.cc
    model/three-gpp-http-header.cc
    model/three-gpp-http-server.cc
    model/three-gpp-http-variables.cc
    model/udp-client.cc
    model/udp-echo-client.cc
    model/udp-echo-server.cc
    model/udp-measurement-client.cc
    model/udp-measurement-server.cc
    model/udp-server.cc
    model/udp-trace-client.cc
    model/voip-application.cc
    model/video-application.cc
    helper/voip-helper.cc
    helper/video-helper.cc
)

set(header_files
    helper/bulk-send-helper.h
    helper/on-off-helper.h
    helper/packet-sink-helper.h
    helper/three-gpp-http-helper.h
    helper/udp-client-server-helper.h
    helper/udp-measurement-client-server-helper.h
    helper/udp-echo-helper.h
    model/application-packet-probe.h
    model/bulk-send-application.h
    model/latency-comp-header.h
    model/onoff-application.h
    model/packet-loss-counter.h
    model/packet-sink.h
    model/seq-ts-echo-header.h
    model/seq-ts-header.h
    model/seq-ts-size-header.h
    model/three-gpp-http-client.h
    model/three-gpp-http-header.h
    model/three-gpp-http-server.h
    model/three-gpp-http-variables.h
    model/udp-client.h
    model/udp-echo-client.h
    model/udp-echo-server.h
    model/udp-measurement-client.h
    model/udp-measurement-server.h
    model/udp-server.h
    model/udp-trace-client.h
    model/voip-application.h
    model/video-application.h
    helper/voip-helper.h
    helper/video-helper.h
)

# link to dependencies
set(libraries_to_link ${libinternet} ${libstats} ${libwifi} ${libreport-manager})

set(test_sources
    test/three-gpp-http-client-server-test.cc
    test/bulk-send-application-test-suite.cc test/udp-client-server-test.cc
)

build_lib("${name}" "${source_files}" "${header_files}" "${libraries_to_link}"
          "${test_sources}"
)
