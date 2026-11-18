"""Generate the documented ROS-free/PCL 1.12 compatibility overlay at build time."""
from pathlib import Path
import sys


def prepare(destination):
    source = Path(__file__).parent / 'upstream'
    for path in source.rglob('*'):
        if not path.is_file():
            continue
        text = path.read_text()
        text = text.replace('#include <ros/ros.h>', '#include <see_logging.h>')
        text = text.replace('boost::shared_ptr', 'std::shared_ptr')
        text = text.replace('boost::make_shared', 'std::make_shared')
        if path.name == 'see_core.cpp':
            # The outer planner selects any reachable proposal, not necessarily
            # core.GetNBV(). Do not blame that frontier for another view's data.
            text = text.replace('if (CheckFrontierPoint(f_idx_)) {', 'if (false) {')
        if path.name == 'pcl_common.h':
            text = '''#pragma once
#define PCL_NO_PRECOMPILE
#include <memory>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/impl/octree_search.hpp>
#include <see_common/pcl_types.h>
''' + text[text.index('typedef pcl::PointCloud<pcl::PointXYZ>'):].replace('#endif // SEE_PCL_COMMON_H', '')
        if path.name == 'pcl_types.h':
            start = text.index('struct MeshTraits')
            end = text.index('/** SEE Point Classifications', start)
            text = text[:start] + text[end:]
            text = text[:text.index('typedef pcl::geometry::PolygonMesh')] + '\n#endif\n'
        target = destination / path.relative_to(source)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text)
    (destination / 'see_logging.h').write_text('''#pragma once
#include <memory>
#include <iomanip>
#include <climits>
#include <cfloat>
#define ROS_INFO_STREAM(x) do {} while (0)
#define ROS_WARN_STREAM(x) do {} while (0)
#define ROS_ERROR_STREAM(x) do {} while (0)
#define ROS_ERROR(x) do {} while (0)
''')


if __name__ == '__main__':
    prepare(Path(sys.argv[1]))
