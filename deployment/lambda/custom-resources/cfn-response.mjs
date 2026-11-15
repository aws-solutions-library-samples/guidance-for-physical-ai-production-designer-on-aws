// FactoryOS public deployment: the CloudFormation response of the stack's own custom resources (CloudFormation calls
// their Lambdas directly; no provider framework). Node standard library only.

import https from 'node:https';

/** PUTs `body` to the pre-signed ResponseURL of `event`. */
export function sendResponse(event, body) {
  const data = JSON.stringify(body);
  return new Promise((resolve, reject) => {
    const req = https.request(event.ResponseURL, {
      method: 'PUT',
      headers: { 'content-type': '', 'content-length': Buffer.byteLength(data) },
    }, (res) => {
      res.resume();
      res.on('end', resolve);
    });
    req.on('error', reject);
    req.end(data);
  });
}

/** The fields every response carries. */
export function responseBase(event, physicalId) {
  return {
    StackId: event.StackId,
    RequestId: event.RequestId,
    LogicalResourceId: event.LogicalResourceId,
    PhysicalResourceId: physicalId,
  };
}

/** An error's name and message only (never the request: a value may be secret). */
export function describeError(e) {
  return `${e?.name || 'Error'}: ${String(e?.message || e).slice(0, 500)}`;
}
