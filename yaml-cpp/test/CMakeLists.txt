file(GLOB test_headers [a-z]*.h)
file(GLOB test_sources [a-z]*.cpp)

add_executable(run-tests
	${test_sources}
	${test_headers}
)
target_link_libraries(run-tests YAML_CPP)

add_test(yaml-reader-test run-tests)
