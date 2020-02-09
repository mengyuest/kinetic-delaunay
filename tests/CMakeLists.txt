set(BINARY test_kinetic_delaunay)

file(GLOB_RECURSE TEST_SOURCES LIST_DIRECTORIES false *.h *.cpp)

add_executable(${BINARY} ${TEST_SOURCES}
        ../include/kinetic-delaunay/controller/collide_event.h
        ../include/kinetic-delaunay/controller/edge_flip_event.h
        ../include/kinetic-delaunay/controller/manager.h
        ../include/kinetic-delaunay/controller/my_event.h
        ../include/kinetic-delaunay/controller/reflect_event.h

        ../include/kinetic-delaunay/data/my_point.h
        ../include/kinetic-delaunay/data/poly.h
        ../include/kinetic-delaunay/data/triangulation.h

        ../src/controller/collide_event.cpp
        ../src/controller/edge_flip_event.cpp
        ../src/controller/manager.cpp
        ../src/controller/my_event.cpp
        ../src/controller/reflect_event.cpp

        ../src/data/my_point.cpp
        ../src/data/poly.cpp
        ../src/data/triangulation.cpp
        )

add_test(NAME ${BINARY}
        COMMAND ${BINARY})

target_link_libraries(${BINARY} PUBLIC
        gtest  ${OpenCV_LIBS})