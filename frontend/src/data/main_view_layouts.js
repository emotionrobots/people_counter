export const DASHBOARD_LAYOUT = {
    rows: 2,
    cols: 3,
    layout: [
        {
            data: 'graphReport',
            color: 'ebony-clay',
            row: 1,
            column: 1,
            size: {
                width: 3,
                height: 1
            }
        },
        {
            data: 'numOccupancy',
            color: 'red-500',
            row: 2,
            column: 1,
            size: {
                width: 1,
                height: 1
            }
        },
        {
            data: 'numLeft',
            color: 'blue-500',
            row: 2,
            column: 2,
            size: {
                width: 1,
                height: 1
            }
        },
        {
            data: 'numEntered',
            color: 'green-500',
            row: 2,
            column: 3,
            size: {
                width: 1,
                height: 1
            }
        },
    ]
}

export const CAMERA_GROUPS_LAYOUT = {
    rows: 2,
    cols: 3,
    layout: [
        {
            data: 'orgSelect',
            color: 'blue-500',
            row: 1,
            column: 1,
            size: {
                width: 1,
                height: 1
            }
        },
        {
            data: 'camGroupSelect',
            color: 'red-500',
            row: 1,
            column: 2,
            size: {
                width: 2,
                height: 1
            }
        },
        {
            data: 'camInfo',
            color: 'green-500',
            row: 2,
            column: 1,
            size: {
                width: 3,
                height: 1
            }
        },
    ]
}

export const PROFILE_LAYOUT = {
    rows: 5,
    cols: 3,
    layout: [
        {
            data: 'profileInfo',
            color: 'ebony-clay',
            row: 1,
            column: 1,
            size: {
                width: 3,
                height: 1
            }
        },
        {
            data: 'myUserInfo',
            color: 'blue-500',
            row: 2,
            column: 1,
            size: {
                width: 1,
                height: 4
            }
        },
        {
            data: 'orgs',
            color: 'red-500',
            row: 2,
            column: 2,
            size: {
                width: 1,
                height: 2
            }
        },
        {
            data: 'camGroups',
            color: 'green-500',
            row: 2,
            column: 3,
            size: {
                width: 1,
                height: 2
            }
        },
        {
            data: 'myCamInfo',
            color: 'ebony-clay',
            row: 4,
            column: 2,
            size: {
                width: 2,
                height: 2
            }
        },
    ]
}