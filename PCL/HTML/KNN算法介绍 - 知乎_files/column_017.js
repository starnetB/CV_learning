(window.webpackJsonp=window.webpackJsonp||[]).push([[35],{1115:function(e,t,n){"use strict";n.r(t);var o=n(29),r=n(9),c=n(11),a=n(12),l=n(13),s=n(6),i=n(0),u=n(2),d=n.n(u),p=n(1),b=n.n(p),h=n(8),f=n(52),O=n(5),v=n(10),j=n(28),m=function(e){return b.a.createElement(j.a,e,b.a.createElement("path",{d:"M17 6.998h3.018c1.098 0 1.975.89 1.982 2.002v10a1.993 1.993 0 0 1-1.987 2H3.98A1.983 1.983 0 0 1 2 19l.009-10.003c0-1.11.873-1.999 1.971-1.999L7 7V5c.016-1.111.822-2 2-2h6c.98 0 1.86.889 2 2v1.998zM9 7h6V5.5s0-.5-.5-.5h-5c-.504 0-.5.5-.5.5V7z",fillRule:"evenodd"}))};m.defaultProps={name:"Company"};var g=m,y=n(1412),w=n(153),M=n.n(w),k=n(457),C=n(157),R=n(81),S=n(262),x=n(358),B=n(214),N=n(7),T=n.n(N),U=n(15),F=n(1177),P=n(251),E=n(231),L=n(63),A=n(1159),q=n(39),_=n(1310),z=n(36);function H(e){var t=function(){if("undefined"==typeof Reflect||!Reflect.construct)return!1;if(Reflect.construct.sham)return!1;if("function"==typeof Proxy)return!0;try{return Boolean.prototype.valueOf.call(Reflect.construct(Boolean,[],(function(){}))),!0}catch(e){return!1}}();return function(){var n,o=Object(s.a)(e);if(t){var r=Object(s.a)(this).constructor;n=Reflect.construct(o,arguments,r)}else n=o.apply(this,arguments);return Object(l.a)(this,n)}}var G=function(e){Object(a.a)(n,e);var t=H(n);function n(){var e;Object(r.a)(this,n);for(var o=arguments.length,c=new Array(o),a=0;a<o;a++)c[a]=arguments[a];return(e=t.call.apply(t,[this].concat(c))).state={modalShown:!1},e.handleSendMessage=function(){var t=e.props,n=t.onModalOpen,o=t.user.allowMessage,r=t.dispatch,c=t.doNotCheckAllowMessageSetting;o||void 0===o&&c?(e.setState({modalShown:!0}),n&&n()):r(Object(q.g)(z.J.blocked,"red")),v.a.trackEvent(Object(L.a)(e),{action:"Message",element:"Button",is_intent:!0})},e.handleCloseModal=function(){var t=e.props.onModalClose;e.setState({modalShown:!1}),t&&t()},e}return Object(c.a)(n,[{key:"render",value:function(){var e=this.props,t=e.user,n=e.preset,o=e.color,r=e.icon,c=e.label,a=e.className,l=e.tooltip,s=e.tooltipPreset,u=this.state.modalShown,d="3d198a56310c02c4a83efb9f4a4c027e"===t.id;return Object(i.c)(i.a,{children:[Object(i.b)(U.c,{preset:n,color:o,className:a,icon:r,label:c,onClick:this.context.authRequired(this.handleSendMessage,d?z.L.serviceAccountMessage:z.L.message),"data-tooltip":l,"aria-label":l,"data-tooltip-preset":s}),Object(i.b)(_.a,{memberHashId:t.id,handleCloseModal:this.handleCloseModal,isModalShow:u})]})}}]),n}(p.Component);G.propTypes={onModalOpen:d.a.func,onModalClose:d.a.func},G.defaultProps={preset:"outline",color:"grey",icon:A.a,label:"发私信",doNotCheckAllowMessageSetting:!1},G.contextTypes={authRequired:d.a.func};var I=Object(h.connect)()(G),J=n(128);function V(e){var t=function(){if("undefined"==typeof Reflect||!Reflect.construct)return!1;if(Reflect.construct.sham)return!1;if("function"==typeof Proxy)return!0;try{return Boolean.prototype.valueOf.call(Reflect.construct(Boolean,[],(function(){}))),!0}catch(e){return!1}}();return function(){var n,o=Object(s.a)(e);if(t){var r=Object(s.a)(this).constructor;n=Reflect.construct(o,arguments,r)}else n=o.apply(this,arguments);return Object(l.a)(this,n)}}var K=function(e){Object(a.a)(n,e);var t=V(n);function n(){var e;Object(r.a)(this,n);for(var o=arguments.length,c=new Array(o),a=0;a<o;a++)c[a]=arguments[a];return(e=t.call.apply(t,[this].concat(c))).state={reportModalShown:!1},e.handleShieldUser=function(){var t=e.props,n=t.user.urlToken,o=t.updateUserBlocking,r=t.showNotification;o(n,!1).then((function(){return r("屏蔽已取消")}))},e}return Object(c.a)(n,[{key:"render",value:function(){var e=this,t=this.props,n=t.user,o=t.user,r=o.urlToken,c=o.isBlocking,a=o.isOrg,l=t.canShowMutualFollowStatus,s=t.className,u=t.buttonClassName,d=t.onModalOpen,p=t.onModalClose,b=this.context.authRequired,h=this.state.reportModalShown,f=T()("MemberButtonGroup",s);return c?Object(i.c)("div",{className:f,children:[Object(i.c)(U.c,{className:u,preset:"primary",color:"red",onClick:this.handleShieldUser,children:[Object(i.b)(F.a,{center:!0,text:!0})," 已屏蔽"]}),!a&&Object(i.c)(U.c,{className:u,onClick:b((function(){e.setState({reportModalShown:!0}),d&&d()}),z.L.report),children:["举报用户",Object(i.b)(E.a,{shown:h,onClose:function(){e.setState({reportModalShown:!1}),p&&p()},type:"member",urlToken:r,zaEnabled:!0})]})]}):Object(i.c)("div",{className:f,children:[Object(i.b)(P.a,{id:r,type:n.type,gender:n.gender,isFollowing:n.isFollowing,isFollowed:n.isFollowed,canShowMutualFollowStatus:l,className:u,preset:P.a.PRESETS.primary}),Object(i.b)(I,{user:n,className:u,onModalOpen:d,onModalClose:p})]})}}]),n}(p.Component);K.contextTypes={authRequired:d.a.func},K.propTypes={user:d.a.object.isRequired,buttonClassName:d.a.string,onModalOpen:d.a.func,onModalClose:d.a.func};var D=Object(h.connect)(null,{updateUserBlocking:J.i,showNotification:q.g})(K);function Q(e){var t=function(){if("undefined"==typeof Reflect||!Reflect.construct)return!1;if(Reflect.construct.sham)return!1;if("function"==typeof Proxy)return!0;try{return Boolean.prototype.valueOf.call(Reflect.construct(Boolean,[],(function(){}))),!0}catch(e){return!1}}();return function(){var n,o=Object(s.a)(e);if(t){var r=Object(s.a)(this).constructor;n=Reflect.construct(o,arguments,r)}else n=o.apply(this,arguments);return Object(l.a)(this,n)}}var W=function(e){Object(a.a)(n,e);var t=Q(n);function n(){return Object(r.a)(this,n),t.apply(this,arguments)}return Object(c.a)(n,[{key:"render",value:function(){var e=this,t=this.props,n=t.user,o=t.user.isOrg,r=t.className,c=t.buttonClassName,a=t.onModalOpen,l=t.onModalClose,s=this.context,u=s.currentUser,d=s.authRequired,p=s.router,b=T()("ProfileButtonGroup",r);return u&&n.urlToken===u.urlToken?Object(i.b)("div",{className:b,children:Object(i.c)(U.c,{color:"blue",className:c,onClick:d((function(){v.a.trackEvent(e,{action:"OpenUrl",element:"Link"},{link:{url:"".concat(location.origin,"/people/edit")}}),p.push("/people/edit")}),z.L.editProfile),children:["编辑",o?"机构":"个人","资料"]})}):Object(i.b)(D,{user:n,canShowMutualFollowStatus:!0,className:b,buttonClassName:c,onModalOpen:a,onModalClose:l})}}]),n}(p.Component);W.contextTypes={router:d.a.object.isRequired,currentUser:d.a.object,authRequired:d.a.func},W.propTypes={user:d.a.object.isRequired,buttonClassName:d.a.string,onModalOpen:d.a.func,onModalClose:d.a.func};var X=W,Y=n(416),Z=n(135),$=n(730),ee=n(731),te=["toggler","user","id","urlToken","mutuals","onRef"];function ne(e){var t=function(){if("undefined"==typeof Reflect||!Reflect.construct)return!1;if(Reflect.construct.sham)return!1;if("function"==typeof Proxy)return!0;try{return Boolean.prototype.valueOf.call(Reflect.construct(Boolean,[],(function(){}))),!0}catch(e){return!1}}();return function(){var n,o=Object(s.a)(e);if(t){var r=Object(s.a)(this).constructor;n=Reflect.construct(o,arguments,r)}else n=o.apply(this,arguments);return Object(l.a)(this,n)}}var oe=function(e,t){return e.map((function(e){return t[e]})).slice(0,3)},re=Object(R.isBrowser)()?M()(oe,(function(e,t){return"".concat(e.join(","),"-").concat(t.length)})):oe,ce=[];var ae=function(e){return Object(i.b)(O.Flex,Object.assign({css:{"&:not(:last-child)":{marginBottom:8}}},e))},le=function(e){Object(a.a)(n,e);var t=ne(n);function n(){var e;Object(r.a)(this,n);for(var o=arguments.length,c=new Array(o),a=0;a<o;a++)c[a]=arguments[a];return(e=t.call.apply(t,[this].concat(c))).state={isLoading:!1,childModalOpened:!1},e.loadUserProfile=function(){var t=e.props,n=t.id,o=t.urlToken;return(0,t.dispatch)(Object(J.g)(o||n,!1))},e.load=function(){var t=e.props,n=t.id;(t.urlToken||n)&&(e.setState({isLoading:!0}),e.loadUserProfile().then((function(){return e.setState({isLoading:!1})})),e.loadMutuals())},e.loadMutuals=function(){var t=e.props,n=t.user,o=t.dispatch,r=t.mutuals;return!(void 0===r?[]:r).length&&e.context.currentUser&&n?o(Object(J.e)(n.urlToken,"mutuals")):Promise.resolve()},e.track=function(){var t=e.props.user;e.hoverCardElement&&t&&(v.a.setModule(e.hoverCardElement,{module:"UserItem"},{card:{content:{type:"User",member_hash_id:t.id}}}),v.a.trackCardShow(e.hoverCardElement))},e.handleOpen=function(){e.load(),e.track()},e}return Object(c.a)(n,[{key:"render",value:function(){var e,t,n=this,r=this.props,c=r.toggler,a=r.user,l=r.id,s=r.urlToken,u=r.mutuals,d=void 0===u?[]:u,p=r.onRef,b=Object(o.default)(r,te),h=a||{},v=h.isOrg,j=h.headline,m=h.avatarUrl,w=h.badge,M=h.badgeV2,R=h.answerCount,N=h.articlesCount,T=h.followerCount,U=h.employments,F=h.coverUrl,P=h.gender,E=h.isFollowed,L=h.isFollowing,A=this.state,q=A.isLoading,_=A.childModalOpened,H=d.length,G=Object(C.d)(M||w),I=G.bestAnswerer,J=G.identity,V="//www.zhihu.com/".concat(v?"org":"people","/").concat(s||l),K=this.context.currentUser,D=K&&a&&K.urlToken===a.urlToken,Q=H>0&&!D,W=U&&U[0];if(null!=J&&J.length)t=Object(i.c)(ae,{children:[Object(i.b)("a",{href:"".concat(z.v,"/account/verification/intro"),target:"_blank",rel:"noopener noreferrer",css:{marginRight:".3em"},children:Object(i.b)(C.a,{})}),Object(i.b)("div",{children:J.slice(0,2).map((function(e){return e.description})).join("，")})]});else if(W){var ne=W.company,oe=W.job;t=Object(i.c)(ae,{children:[(ne||oe)&&Object(i.b)(O.Box,{as:"span",mr:".3em",color:"GBL05A",children:Object(i.b)(g,{text:!0,center:!0})}),Object(i.c)("div",{children:[ne&&ne.name,ne&&oe&&Object(i.b)(O.Box,{display:"inline-block",width:"1px",height:"10px",mx:"8px",bg:"GBK09A"}),oe&&oe.name]})]})}else t=null;return Object(i.c)(x.b,Object.assign(Object.assign({toggler:c,onOpen:this.handleOpen,isLoading:q,globalClose:!_,stickHover:_,preventCloseOnTarget:!0},b),{},{onRef:function(e){p&&"function"==typeof p&&p(e),n.hoverCardElement=e},children:[Object(i.b)(x.a,{coverUrl:F,title:Object(i.c)("span",{css:{display:"flex"},children:[Object(i.b)(Y.a,{user:a,noHoverCard:!0}),Object(i.b)(ee.a,{isFollowed:E,isFollowing:L})]}),subtitle:j&&Object(i.b)(B.a,{inline:!0,html:j}),avatarUrl:m}),(t||I||Q)&&Object(i.c)("div",{children:[t,Boolean(null==I||null===(e=I.topics)||void 0===e?void 0:e.length)&&Object(i.c)(ae,{children:[Object(i.b)("span",{css:{marginRight:".3em"},children:Object(i.b)(C.b,{})}),Object(i.c)("div",{children:[Object(i.b)($.a,{expandable:!1,noHoverCard:!0,topics:I.topics,user:a}),"的优秀答主"]})]}),Q&&Object(i.c)(ae,{color:"GBL05A",children:[Object(i.b)(O.Box,{mr:".3em",children:Object(i.b)(y.a,{text:!0,center:!0})}),Object(i.b)("div",{children:Object(i.c)(k.CJKSpace,{children:[d.map((function(e,t){return Object(i.c)(f.Link,{to:"//www.zhihu.com/".concat(v?"org":"people","/").concat(e.urlToken),target:"_blank",children:[e.name,t<H-1&&"、"]},t)})),"也关注了",z.D[P]]})})]})]}),Object(i.c)("div",{children:[Object(i.c)(S.a,{children:[Object(i.b)(S.a.Item,{tag:Z.a,preset:"plain",to:"".concat(V,"/answers"),name:"回答",value:R,target:"_blank"}),Object(i.b)(S.a.Item,{tag:Z.a,preset:"plain",to:"".concat(V,"/posts"),name:"文章",value:N,target:"_blank"}),Object(i.b)(S.a.Item,{tag:Z.a,preset:"plain",to:"".concat(V,"/followers"),name:"关注者",value:T,target:"_blank"})]}),a&&!D&&Object(i.b)(X,{className:"HoverCard-buttons",user:a,onModalOpen:function(){n.hoverCardElement.style.opacity="0",n.setState({childModalOpened:!0})},onModalClose:function(){return n.setState({childModalOpened:!1})}})]})]}))}}]),n}(p.PureComponent);le.contextTypes={currentUser:d.a.object};t.default=Object(h.connect)((function(e,t){var n=t.id,o=t.urlToken,r=e.entities,c=e.people.mutualsByUser,a=r.users[o]||Object.values(r.users).find((function(e){return e.id===n}))||t.user,l=c[o||n]||{ids:ce,isFetching:!1},s=l.ids,i=l.isFetching;return{user:a,mutuals:re(s,r.users),isFetching:i}}))(le)},1412:function(e,t,n){"use strict";var o=n(1),r=n.n(o),c=n(28),a=function(e){return r.a.createElement(c.a,e,r.a.createElement("path",{d:"M9.676 11.252c-.243.433-.542.85-.494 1.237.189 1.509 2.938 1.807 3.746 2.816.59.736.835 1.852.892 3.725.01.303 0 .97-.867.97H1.933c-.915 0-.925-.643-.915-.936.062-1.877.292-3.02.895-3.76.812-.994 3.743-1.267 3.872-2.872.03-.361-.246-.74-.493-1.18C3.917 8.802 3.39 4 7.466 4s3.506 4.941 2.21 7.252zm8.073.79l.002.641c1.57 1.766 4.47 1.307 4.851 3.166.125.61.328 1.538.384 3.232.009.284.014.919-.905.919-.92 0-5.523-.021-6.875-.019 0-2.806-.144-5.987-2.707-6.285.537-.331 1.725-.504 2.178-1.013l.002-.64c-.881.118-2.679-.541-2.679-.541.61-.64.747-1.387 1.192-4.317.355-2.93 2.835-2.903 3.022-2.903.187 0 2.667-.027 3.028 2.903.439 2.93.577 3.677 1.186 4.317 0 0-1.797.659-2.68.54z",fillRule:"evenodd"}))};a.defaultProps={name:"Users"},t.a=a}}]);
//# sourceMappingURL=column.user-hover-card.66a73881d63daaef1b5f.js.map