import React, { Component } from 'react'
import InfoWidget from '../components/InfoWidget'
import { DASHBOARD_LAYOUT } from '../data/main_view_layouts';

class SubPage extends Component {
    constructor(props){
        super()
        this.state = props.layout;

        if(this.state == null){
            this.state = DASHBOARD_LAYOUT
        }
    }

    render() {
        return (
            <div className={"w-full grid grid-flow-col grid-rows-" + this.state.rows + " grid-cols-" + this.state.cols + " gap-4"}>
                {this.state.layout.map((val, index) => 
                    <div key={index} className={"col-span-" + val.size.width + " row-span-" + val.size.height + " row-start-" + val.row + " col-start-" + val.column}>
                        <InfoWidget bgColor={val.color} data={val.data}></InfoWidget>
                    </div>
                    )}
            </div>
        )
    }
}

export default SubPage
