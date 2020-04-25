#pragma once

#include <cstdint>
#include <string>
#include <vector>

using namespace std;


namespace kitti_bev
{
    struct KittiSequence_t
    {
        KittiSequence_t(const string& kId, const string& kName, const size_t kStart,
            const size_t kEnd, bool useSync)
        : _id(kId),
            _name(kName),
            _start(kStart),
            _end(kEnd)
        {
            if (useSync)
                _name += "_sync";
        }

        const string& id() const { return this->_id; }
        const string& name() const { return this->_name; }
        size_t start() const { return this->_start; }
        size_t end() const { return this->_end; }
        size_t getNumScans() const { return this->_end - this->_start + 1; }

        static string to_string(const size_t id)
        {
            ostringstream ss;
            ss << setw(10) << setfill('0') << id;
            return ss.str();
        }

        static KittiSequence_t getSeqInfo(const size_t id)
        {
            static vector<KittiSequence_t> seqs = {
                KittiSequence_t("00", "2011_10_03_drive_0027",    0, 4540, true),
                KittiSequence_t("01", "2011_10_03_drive_0042",    0, 1100, true),
                KittiSequence_t("02", "2011_10_03_drive_0034",    0, 4660, true),
                KittiSequence_t("03", "2011_09_26_drive_0067",    0,  800, true),
                KittiSequence_t("04", "2011_09_30_drive_0016",    0,  270, true),
                KittiSequence_t("05", "2011_09_30_drive_0018",    0, 2760, true),
                KittiSequence_t("06", "2011_09_30_drive_0020",    0, 1100, true),
                KittiSequence_t("07", "2011_09_30_drive_0027",    0, 1100, true),
                KittiSequence_t("08", "2011_09_30_drive_0028", 1100, 5170, true),
                KittiSequence_t("09", "2011_09_30_drive_0033",    0, 1590, true),
                KittiSequence_t("10", "2011_09_30_drive_0034",    0, 1200, true)
            };
            return seqs[id];
        }

    private:
        string _id, _name;
        size_t _start, _end;
    };
}
