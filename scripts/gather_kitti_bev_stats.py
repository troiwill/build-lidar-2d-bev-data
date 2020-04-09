import argparse
import os
import pandas
from pathlib import Path


def main():
    # Parse the command-line arguments.
    parser = argparse.ArgumentParser()
    parser.add_argument('kitti_bev_root', help='Root directory for Kitti BEV data.')
    args = parser.parse_args()

    #
    print('Searching for Kitti BEV info files...', end='')
    bev_root = Path(args.kitti_bev_root)
    infofile_list = sorted([ str(p) for p in bev_root.glob('**/**/info.txt') ])
    print('Done! Found {} files.'.format(len(infofile_list)), end='\n\n')

    date_set = set()
    info_list = list()
    col_names = [ 'scanId', 'cpy', 'cpx', 'yaw', 'res', 'delta_dist', 'acc_dist' ]
    NB_FILES = len(infofile_list)
    for i, infofile in enumerate(infofile_list):
        # Read info file.
        info = pandas.read_csv(infofile, names=col_names, skiprows=[ 0 ])

        # Extract the information from the info file.
        infodirs = infofile.split('/')[:-1]
        datedir, drivedir = infodirs[-2:]
        nb_scans = len(info)
        total_drive_dist = info.at[nb_scans - 1, 'acc_dist']
        date_set = date_set | set([ datedir ])

        info_list.append(list((datedir, drivedir, nb_scans, total_drive_dist)))
        print('Progress ==> {:.2f}%\r'.format((i+1) / NB_FILES * 100.), end='')
    #end for

    df = pandas.DataFrame(info_list, columns=[ 'date', 'drive', 'nb_scans', 'drive_dist' ])
    for datename in sorted(list(date_set)):

        date_df = df[df['date'] == datename]
        acc_scans = date_df.get('nb_scans').sum()
        acc_total_dist = date_df.get('drive_dist').sum()
        nb_date_date = len(date_df)

        print('Accumulated Date Info: ' + datename)
        print('  # of drives  -> {}'.format(nb_date_date))
        print('  # of scans   -> {}'.format(acc_scans))
        print('  acc distance -> {}'.format(acc_total_dist))
        print('', end='\n\n')
    #end for
#end def

if __name__ == '__main__':
    main()
#end if
