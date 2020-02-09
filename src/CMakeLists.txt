add_executable(kinetic_delaunay
        main.cpp
        ../include/kinetic-delaunay/controller/collide_event.h
        ../include/kinetic-delaunay/controller/edge_flip_event.h
        ../include/kinetic-delaunay/controller/manager.h
        ../include/kinetic-delaunay/controller/my_event.h
        ../include/kinetic-delaunay/controller/reflect_event.h

        ../include/kinetic-delaunay/data/my_point.h
        ../include/kinetic-delaunay/data/poly.h
        ../include/kinetic-delaunay/data/triangulation.h

        controller/collide_event.cpp
        controller/edge_flip_event.cpp
        controller/manager.cpp
        controller/my_event.cpp
        controller/reflect_event.cpp

        data/my_point.cpp
        data/poly.cpp
        data/triangulation.cpp
        )

#add_subdirectory(data)
#add_subdirectory(controller)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS}
#        data
#        controller
        )