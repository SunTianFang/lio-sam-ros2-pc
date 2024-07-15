include_directories(
	../Tools/include
    include
    ../ThirdParty/eigen3
    )


set ( Geometry_SRCS
	src/Angle.cpp
	src/Arc.cpp
	src/Bezier.cpp
	src/Circle.cpp
	src/Direct.cpp
	src/Ellipse.cpp
	src/Frame.cpp
	src/Line.cpp
	src/LineBase.cpp
	src/MultiSegLine.cpp
	src/NewCurve.cpp
	src/Pnt.cpp
	src/PolyRegion.cpp
	src/Posture.cpp
	src/DataRange.cpp
	src/Rectangle.cpp
	src/ScrnRef.cpp
	src/Spline.cpp
	src/Scp.cpp
	src/Spp.cpp
	src/Transfor.cpp
	src/AppArea.cpp
	src/AffinePosture.cpp
	src/MatchInfo.cpp
	)

add_library ( Geometry SHARED ${Geometry_SRCS}  )
link_libraries(${GLIB_LIB_DIR})
