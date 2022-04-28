import React, { useContext } from 'react'
import InfoWidget from '../components/InfoWidget'
import { DASHBOARD_LAYOUT } from '../data/main_view_layouts';
import { UserContext } from '../components/Contexts/UserContext';

function SubPage(props) {
    let userContext = useContext(UserContext)
    let layout = props.layout || DASHBOARD_LAYOUT

    return (
        (userContext.error !== undefined && userContext.error === 0) ? <div className={"h-full w-full grid grid-flow-col grid-rows-" + layout.rows + " grid-cols-" + layout.cols + " gap-4"}>
            {layout.layout.map((val, index) =>
                <div key={index} className={"h-full col-span-" + val.size.width + " row-span-" + val.size.height + " row-start-" + val.row + " col-start-" + val.column}>
                    <InfoWidget bgColor={val.color} data={val.data}></InfoWidget>
                </div>
            )
            }
        </div> :
            <div className='flex h-full w-full p-5 justify-center items-center text-center text-white font-bold text-xl'>
                There was an error retrieving your user information. Your connection may be unstable or the servers are temporarily down.
            </div>
    )
}
export default SubPage
